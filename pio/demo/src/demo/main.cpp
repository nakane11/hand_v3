#include <M5AtomS3.h>
#include <IcsHardSerialClass.h>
#include <SBUS.h>
#include <button.h>
#include <VacuumEsp.h>
#include <ADS7828.h>
#include <mode_manager.h>

// === ICS === //
const byte EN_PIN = 6;
const byte RX_PIN = 5;
const byte TX_PIN = 38;
const long BAUDRATE = 1250000;
const int TIMEOUT = 20;
const uint8_t kondo_ids[5] = {0,1,2,3,5};
IcsHardSerialClass krs(&Serial1, EN_PIN, BAUDRATE, TIMEOUT);
VacuumEsp vacuum(krs);

// === Futaba === //
const byte RX_PIN_FUTABA = 10;
const byte TX_PIN_FUTABA = 39;
const long BAUDRATE_FUTABA = 100000;
SBUS sbus(&Serial2, RX_PIN_FUTABA, TX_PIN_FUTABA, BAUDRATE_FUTABA);

// ===Smoothing=== //
#define NUM_SERVOS 11
int targetAngles[NUM_SERVOS];    // 目標角度
int currentAngles[NUM_SERVOS];   // 現在の角度
float totalTime[NUM_SERVOS]; // 各サーボごとの動作時間
float startTime[NUM_SERVOS]; // 各サーボの動作開始時間
bool updateCompleteFlag = true;  // 更新完了フラグ

// ===FSR=== //
#define NUM_SENSOR_CHANNELS 8
#define NUM_SENSOR_DEVICES 4
ADS7828 ads[NUM_SENSOR_DEVICES] = {ADS7828(0x48), ADS7828(0x49), ADS7828(0x4A), ADS7828(0x4B)};
uint8_t force_data[NUM_SENSOR_DEVICES*NUM_SENSOR_CHANNELS];

ModeManager modeManager;

void initialize_servo(){
  //futaba 初期姿勢をcurrentangleとtargetangleに入れてsmoothingせずに送る
  currentAngles[1] = 60;
  currentAngles[2] = -60;
  currentAngles[4] = 60;
  currentAngles[3] = -60;
  currentAngles[0] = -60;
  currentAngles[5] = -60;
  for (int i = 0; i < 6; i++) {
    targetAngles[i] = currentAngles[i];
    sbus.setServoAngle(i,currentAngles[i]);
  }
  sbus.begin();
  sbus.sendSbusData();
  //kondo 現在角を読んでcurrentangleとtargetangleに入れる
  for (int i = 6; i < NUM_SERVOS; i++) {
    int pos = krs.getPos(i-6);
    currentAngles[i] = pos;
    targetAngles[i] = pos;
  }
}

void updateServoAngles(int i, int *currentAngles, int *targetAngles, float *startTime, float *totalTime, bool &allServosUpdated, void (*setServoAngle)(int, float)) {
  if (abs(targetAngles[i] - currentAngles[i]) < 1) {
    delay(1);
    return;
  }
  
  float t = (millis() / 1000.0) - startTime[i]; // 経過時間 (秒)
  if (t >= totalTime[i]) {
    // 目標に到達した場合は、最終角度に設定
    currentAngles[i] = targetAngles[i];
    setServoAngle(i, currentAngles[i]);
  } else {
    float T = totalTime[i];
    float tau = t / T;
    float tau3 = tau * tau * tau;
    float tau4 = tau3 * tau;
    float tau5 = tau4 * tau;
    currentAngles[i] = currentAngles[i] + (targetAngles[i] - currentAngles[i]) *
      (10 * tau3 - 15 * tau4 + 6 * tau5);
    setServoAngle(i, currentAngles[i]);
    allServosUpdated = false;
  }
}

void smoothUpdate(void *parameter) {
  while (true) {
    if (updateCompleteFlag) {
      delay(1);
      continue;
    }
    bool allServosUpdated = true;

    // sbus サーボの更新
    for (int i = 0; i < 6; i++) {
      updateServoAngles(i, currentAngles, targetAngles, startTime, totalTime, allServosUpdated, [](int idx, float angle) {
        sbus.setServoAngle(idx, angle);
      });
    }
    if (!allServosUpdated) {
      sbus.sendSbusData();
      delay(3);
    }
    // krs サーボの更新
    for (int i = 6; i < NUM_SERVOS; i++) {
      updateServoAngles(i, currentAngles, targetAngles, startTime, totalTime, allServosUpdated, [](int idx, float angle) {
        krs.setPos(kondo_ids[idx - 6], angle);
      });
    }

    // 全てのサーボの更新が完了した場合
    if (allServosUpdated) {
      updateCompleteFlag = true; // 更新完了フラグを立てる
    }
    delay(2);
  }
}

void sensorUpdate(void *parameter){
  while (true) {
    // M5.Lcd.setCursor(0,0);
    for(int i = 0; i < NUM_SENSOR_DEVICES; i++){
      for(int j = 0; j<NUM_SENSOR_CHANNELS; j++){
        force_data[i*NUM_SENSOR_CHANNELS+j] = ads[i].getValue(j);
        // M5.Lcd.print(force_data[i*NUM_SENSOR_CHANNELS+j]);
        // M5.Lcd.print(" ");
        delay(2);
      }
      // M5.Lcd.println();
    }
  }
}

void setup(){
  M5.begin();
  // M5.Lcd.setRotation(3);  // 画面向き設定（USB位置基準 0：下/ 1：右/ 2：上/ 3：左）
  // M5.Lcd.setTextSize(2);  // 文字サイズ（整数倍率）
  
  Serial1.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN, false, TIMEOUT);
  krs.begin();
  for(int i=1;i<5;i++){
    krs.setSpd(i,80);
  }
  initialize_servo();
  Wire.begin(8,7);
  xTaskCreatePinnedToCore(sensorUpdate, "Sensor Update", 2048, NULL, 22, NULL, 0);
  xTaskCreatePinnedToCore(smoothUpdate, "Smooth Update", 2048, NULL, 8, NULL, 0);
  xTaskCreatePinnedToCore(ButtonTask, "Button Task", 2048, NULL, 24, NULL, 1);
  xTaskCreatePinnedToCore(VacuumEsp::pressureControlLoopWrapper, "Pressure control loop", 2048, &vacuum, 10, NULL, 0);
}

void loop(){
  M5.update();  //本体ボタン状態更新
  modeManager.updateMode(force_data);
  modeManager.processTask(targetAngles, totalTime, startTime, updateCompleteFlag);
  if(currentButtonState==PRESSED){
    if(vacuum.pressure_control_running){
      // M5.Lcd.setCursor(0,60);
      // M5.Lcd.println("release");
      vacuum.release = true;
    }else{
      // M5.Lcd.setCursor(0,60);
      // M5.Lcd.println("vacuum");
      vacuum.pressure_control_running=true;
    }
  }else if(currentButtonState==SINGLE_CLICK){
  }
  currentButtonState = NOT_CHANGED;
  delay(10);
}
