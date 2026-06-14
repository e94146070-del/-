
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// 定義 BLE 的服務與特徵 UUID
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID                "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID                "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

// =========================================================================
// 1. 🚀 安全腳位與全域變數宣告
// =========================================================================
const int enableLeft  = 16; const int motorLeft1  = 4;  const int motorLeft2  = 17; 
const int motorRight1 = 14; const int motorRight2 = 32; const int enableRight = 13; 
const int SENSOR_PINS[5] = {25, 26, 27, 15, 22}; 

// 🚥 狀態指示燈腳位
const int LED_R = 19; 
const int LED_G = 21; 

enum CarState { STATE_SAFE, STATE_AUTO };
CarState currentStatus = STATE_SAFE; 

// =========================================================================
// 參數區 (黃金參數完全還原)
// =========================================================================
double Kp = 25.0; 
double Kd = 18.0;    
const int CRUISE_SPEED = 255; 
const int ROCKET_SPEED = 255; 
int currentBaseSpeed = CRUISE_SPEED; 

unsigned long straightStartTime = 0; bool isStraight = false;
unsigned long autoStartTime = 0; const unsigned long KICKSTART_DELAY = 80; 
unsigned long lastSensorUpdateTime = 0; 

// 🏁 🚀 鎖定機制的過彎特調參數
const int ARC_OUTER_SPEED = 255; 
const int ARC_INNER_SPEED = -150;    
const int SEARCH_OUTER_SPEED = 255; 
const int SEARCH_INNER_SPEED = -200;  
const unsigned long SENSOR_PERIOD = 2; 

// ⏱️ 【非阻塞延遲過彎控制組】
enum TurnStage { STAGE_IDLE, STAGE_DELAY, STAGE_LOCK };
TurnStage currentTurnStage = STAGE_IDLE;
unsigned long turnStageStartTime = 0; 
int pendingTurnDirection = 0; 
const unsigned long DELAY_BEFORE_TURN = 30; 
const unsigned long TURN_LOCK_DURATION = 55; 

// 馬達物理偏差補償比例
double LEFT_MOTOR_COMPENSATE = 0.97; 
double RIGHT_MOTOR_COMPENSATE = 1.00; 

double smoothedError = 0; 
int lastError = 0; 
int lastValidDirection = 0; 
int lastOuterSensorMemory = 0; 

// =========================================================================
// 2. 驅動核心與封裝管理 (🛠️ 順序提前以解決 "not declared in this scope" 錯誤)
// =========================================================================
void moveCar(int leftSpeed, int rightSpeed) {
  if (leftSpeed != 0)  leftSpeed  = (int)(leftSpeed * LEFT_MOTOR_COMPENSATE);
  if (rightSpeed != 0) rightSpeed = (int)(rightSpeed * RIGHT_MOTOR_COMPENSATE);
  
  if (leftSpeed > 0 && leftSpeed < 75)   leftSpeed = 75;
  if (rightSpeed > 0 && rightSpeed < 75) rightSpeed = 75;
  
  leftSpeed = constrain(leftSpeed, -255, 255); 
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  if (leftSpeed > 0) { digitalWrite(motorLeft1, LOW); digitalWrite(motorLeft2, HIGH); analogWrite(enableLeft, leftSpeed); }
  else if (leftSpeed < 0) { digitalWrite(motorLeft1, HIGH); digitalWrite(motorLeft2, LOW); analogWrite(enableLeft, abs(leftSpeed)); }
  else { digitalWrite(motorLeft1, HIGH); digitalWrite(motorLeft2, HIGH); analogWrite(enableLeft, 255); }

  if (rightSpeed > 0) { digitalWrite(motorRight1, LOW); digitalWrite(motorRight2, HIGH); analogWrite(enableRight, rightSpeed); }
  else if (rightSpeed < 0) { digitalWrite(motorRight1, HIGH); digitalWrite(motorRight2, LOW); analogWrite(enableRight, abs(rightSpeed)); }
  else { digitalWrite(motorRight1, HIGH); digitalWrite(motorRight2, HIGH); analogWrite(enableRight, 255); }
}

int getPositionError() {
  int l2 = !digitalRead(SENSOR_PINS[0]); int l1 = !digitalRead(SENSOR_PINS[1]); 
  int m  = !digitalRead(SENSOR_PINS[2]); int r1 = !digitalRead(SENSOR_PINS[3]); int r2 = !digitalRead(SENSOR_PINS[4]); 
  int count = l2 + l1 + m + r1 + r2;
  if (count == 0) return 0; 
  int rawError = ((l2 * -5) + (l1 * -4) + (m * 0) + (r1 * 4) + (r2 * 5)) / count;
  smoothedError = (smoothedError * 0.1) + (rawError * 0.9); 
  if (rawError < -1) lastValidDirection = -1; 
  if (rawError > 1)  lastValidDirection = 1;
  return (int)smoothedError;
}

bool executeDelayThenTurn() {
  if (currentTurnStage == STAGE_IDLE) return false;
  unsigned long elapsed = millis() - turnStageStartTime;
  if (currentTurnStage == STAGE_DELAY) {
    if (elapsed < DELAY_BEFORE_TURN) { moveCar(CRUISE_SPEED, CRUISE_SPEED); return true; } 
    else { currentTurnStage = STAGE_LOCK; turnStageStartTime = millis(); }
  }
  if (currentTurnStage == STAGE_LOCK) {
    if (elapsed < TURN_LOCK_DURATION) {
      if (pendingTurnDirection == -1) moveCar(ARC_INNER_SPEED, ARC_OUTER_SPEED); 
      else                            moveCar(ARC_OUTER_SPEED, ARC_INNER_SPEED); 
      return true;
    } else { currentTurnStage = STAGE_IDLE; return false; }
  }
  return false;
}

// =========================================================================
// 📥 BLE 訊號接收核心連線鎖 (🛠️ 修正 rxValue 為 String 類型的核心架構)
// =========================================================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue(); // 🛠️ 核心修正：符合新版 ESP32 庫的 String 宣告
        if (rxValue.length() > 0) {
            char cmd = rxValue[0]; // 讀取第一個控制字元
            
            // 🚀 收到 'S'：開始導航
            if (cmd == 'S' && currentStatus == STATE_SAFE) {
                currentStatus = STATE_AUTO; 
                autoStartTime = millis(); 
                lastError = 0; smoothedError = 0; lastOuterSensorMemory = 0;
                currentTurnStage = STAGE_IDLE;
                
                digitalWrite(LED_G, LOW); 
                digitalWrite(LED_R, HIGH); // 🔴 切換紅燈
                Serial.println("BLE 指令：開始導航！");
            }
            // 🛑 收到 'E'：緊急煞車
            else if (cmd == 'E') {
                currentStatus = STATE_SAFE;
                moveCar(0, 0); // 馬達煞死
                
                digitalWrite(LED_R, LOW); 
                digitalWrite(LED_G, HIGH); // 🟢 回復綠燈
                Serial.println("BLE 指令：緊急煞車！");
            }
        }
    }
};

// =========================================================================
// 3. 初始化與主迴圈
// =========================================================================
void setup() {
  Serial.begin(115200);
  
  // 初始化燈號腳位並點亮綠燈
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT);
  digitalWrite(LED_R, LOW); 
  digitalWrite(LED_G, HIGH); 
  
  // 初始化 BLE 裝置
  BLEDevice::init("2026_Geometry_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 建立 BLE 序列埠模擬服務
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE 藍牙模組編譯成功且啟動！等待 APP 直接連線...");

  // 初始化硬體腳位
  pinMode(motorLeft1, OUTPUT); pinMode(motorLeft2, OUTPUT); pinMode(enableLeft, OUTPUT);
  pinMode(motorRight1, OUTPUT); pinMode(motorRight2, OUTPUT); pinMode(enableRight, OUTPUT);
  for (int i = 0; i < 5; i++) pinMode(SENSOR_PINS[i], INPUT);
  
  currentStatus = STATE_SAFE; lastError = 0; smoothedError = 0; currentTurnStage = STAGE_IDLE; lastOuterSensorMemory = 0; moveCar(0, 0);
  Serial.println("--- 2026 GEOMETRY RACING SYSTEM READY ---");
}

void loop() {
  // BLE 斷線自動恢復廣播機制
  static bool wasConnected = false;
  if (!deviceConnected && wasConnected) {
      delay(500);
      pServer->startAdvertising(); 
      Serial.println("偵測到中斷，重新啟用 BLE 廣播...");
      wasConnected = false;
  }
  if (deviceConnected && !wasConnected) { wasConnected = true; }

  // 模式 1：安全靜止模式
  if (currentStatus == STATE_SAFE) { 
    moveCar(0, 0); 
    return; 
  } 
  
  // 模式 2：自動循跡導航模式
  else if (currentStatus == STATE_AUTO) {
    unsigned long runTime = millis() - autoStartTime;
    if (runTime < KICKSTART_DELAY) { 
      int softSpeed = map(runTime, 0, KICKSTART_DELAY, 180, 255);
      moveCar(softSpeed, softSpeed); return; 
    }

    if (millis() - lastSensorUpdateTime >= SENSOR_PERIOD) {
      lastSensorUpdateTime = millis();
      if (executeDelayThenTurn()) return; 

      int l2 = !digitalRead(SENSOR_PINS[0]); int l1 = !digitalRead(SENSOR_PINS[1]); int m = !digitalRead(SENSOR_PINS[2]);
      int r1 = !digitalRead(SENSOR_PINS[3]); int r2 = !digitalRead(SENSOR_PINS[4]);

      if (l2 == 1 && r2 == 0) lastOuterSensorMemory = -1; 
      else if (r2 == 1 && l2 == 0) lastOuterSensorMemory = 1;  

      // ⬜ 1. 全白記憶盲尋
      if (l2 == 0 && l1 == 0 && m == 0 && r1 == 0 && r2 == 0) {
        isStraight = false;
        int finalSearchDir = (lastOuterSensorMemory != 0) ? lastOuterSensorMemory : lastValidDirection;
        if (finalSearchDir == -1) moveCar(SEARCH_INNER_SPEED, SEARCH_OUTER_SPEED); 
        else                      moveCar(SEARCH_OUTER_SPEED, SEARCH_INNER_SPEED); 
        return; 
      }

      // ⬛ 2. 全黑維持上一動作
      if (l2 == 1 && l1 == 1 && m == 1 && r1 == 1 && r2 == 1) return; 

      // 🛑 3. 直角與銳角過彎狀態機觸發
      bool leftTurnTrigger  = (l2 == 1 && r2 == 0) || (l1 == 1 && l2 == 1);
      bool rightTurnTrigger = (r2 == 1 && l2 == 0) || (r1 == 1 && r2 == 1);

      if (leftTurnTrigger || rightTurnTrigger) {
        isStraight = false;
        if (currentTurnStage == STAGE_IDLE) {
          currentTurnStage = STAGE_DELAY; turnStageStartTime = millis();      
          if (leftTurnTrigger) { pendingTurnDirection = -1; lastValidDirection = -1; lastOuterSensorMemory = -1; lastError = -5; } 
          else { pendingTurnDirection = 1; lastValidDirection = 1; lastOuterSensorMemory = 1; lastError = 5; }
        }
        return;
      }

      int error = getPositionError();

      // 🎯 4. 大直線 + 直線末端收油防噴飛機制
      if (abs(error) <= 1) { 
        if (!isStraight) { straightStartTime = millis(); isStraight = true; }
        int speed = ROCKET_SPEED;
        if (millis() - straightStartTime > 250) speed = 175; // 主動收油
        else if (millis() - straightStartTime <= 40) speed = CRUISE_SPEED;
        int straightAdjust = (int)(error * 5.0);
        moveCar(speed - straightAdjust, speed + straightAdjust);
      } 
      // 📉 5. 彎道極速 PD 控制
      else { 
        isStraight = false;
        currentBaseSpeed = (abs(error) >= 3) ? (CRUISE_SPEED - 65) : CRUISE_SPEED;
        int turnAdjust = -(int)((Kp * error) + (Kd * (error - lastError)));
        turnAdjust = constrain(turnAdjust, -210, 210); 
        moveCar(currentBaseSpeed - turnAdjust, currentBaseSpeed + turnAdjust);
      }
      lastError = error;
    }
  } 
}