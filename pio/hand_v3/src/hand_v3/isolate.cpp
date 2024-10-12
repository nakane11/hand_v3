#include <M5AtomS3.h>
#include <IcsHardSerialClass.h>
#include <OneButton.h>
#include <SBUS.h>

enum ServoState {
  FREE,
  HOLD
};
ServoState currentServoState = FREE;

// ==== Button ==== //
constexpr int BUTTON_PIN = 41;
OneButton btn(BUTTON_PIN, true, false);
enum ButtonState {
  NOT_CHANGED,
  SINGLE_CLICK,
  DOUBLE_CLICK,
  TRIPLE_CLICK,
  QUADRUPLE_CLICK,  // 4 clicks
  QUINTUPLE_CLICK,  // 5 clicks
  SEXTUPLE_CLICK,   // 6 clicks
  SEPTUPLE_CLICK,   // 7 clicks
  OCTUPLE_CLICK,    // 8 clicks
  NONUPLE_CLICK,    // 9 clicks
  DECUPLE_CLICK,    // 10 clicks
  PRESSED,
  RELEASED,
  RESET,
  BUTTON_STATE_COUNT
};
ButtonState currentButtonState = RESET;

static void handleClick() {
  currentButtonState = SINGLE_CLICK;
}

static void handleDoubleClick() {
  currentButtonState = DOUBLE_CLICK;
}

static void handleMultiClick() {
  int n = btn.getNumberClicks();
  switch (n) {
  case 1: currentButtonState = SINGLE_CLICK; break;
  case 2: currentButtonState = DOUBLE_CLICK; break;
  case 3: currentButtonState = TRIPLE_CLICK; break;
  case 4: currentButtonState = QUADRUPLE_CLICK; break;
  case 5: currentButtonState = QUINTUPLE_CLICK; break;
  case 6: currentButtonState = SEXTUPLE_CLICK; break;
  case 7: currentButtonState = SEPTUPLE_CLICK; break;
  case 8: currentButtonState = OCTUPLE_CLICK; break;
  case 9: currentButtonState = NONUPLE_CLICK; break;
  case 10: currentButtonState = DECUPLE_CLICK; break;
  default: currentButtonState = DECUPLE_CLICK; // Handle case where n > 10
  }
}

static void handleLongPress() {
  currentButtonState = PRESSED;
}

static void handleLongPressEnd() {
  currentButtonState = RELEASED;
}

void ButtonTask(void *parameter) {
  btn.setClickMs(200);  // Timeout used to distinguish single clicks from double clicks. (msec)
  btn.attachClick(handleClick);
  btn.attachDoubleClick(handleDoubleClick);
  btn.attachMultiClick(handleMultiClick);
  btn.attachLongPressStart(handleLongPress);
  btn.attachLongPressStop(handleLongPressEnd);// Initialize last receive time

  while (true) {
    btn.tick();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// === ICS Servo === //
const byte EN_PIN = 6;
const byte RX_PIN = 5;
const byte TX_PIN = 38;
const long BAUDRATE = 1250000;
const int TIMEOUT = 20;
IcsHardSerialClass krs(&Serial1, EN_PIN, BAUDRATE, TIMEOUT);

// === FUTABA Servo === //
const byte RX_PIN_FUTABA = 10;
const byte TX_PIN_FUTABA = 39;
const long BAUDRATE_FUTABA = 100000;
SBUS sbus(&Serial2, RX_PIN_FUTABA, TX_PIN_FUTABA, BAUDRATE_FUTABA);

#define NUM_SERVOS 11
int targetAngles[NUM_SERVOS];    // 目標角度
int currentAngles[NUM_SERVOS];   // 現在の角度
float totalTime[NUM_SERVOS]; // 各サーボごとの動作時間
float startTime[NUM_SERVOS]; // 各サーボの動作開始時間
bool updateCompleteFlag = true;  // 更新完了フラグ
// const float alpha = 0.01; // 目標角度に対する重み
// const float dt = 0.1; // タイムステップ (秒)

void reset_arm()
{
  for(int i=1;i<5;i++){
    krs.setPos(i, 7500);
    delay(1);
  }
  currentServoState = HOLD;
}

void free_arm()
{
  for(int i=1;i<5;i++){
    krs.setFree(i);
    delay(1);
  }
  currentServoState = FREE;
}

void hold_arm()
{
  for(int i=1;i<5;i++){
    krs.setPos(i, krs.getPos(i));
    delay(1);
  }
  currentServoState = HOLD;
}

void wave_arm()
{
  for(int i=0;i<2;i++){
    for(int j=7500;j<8500;j=j+2){
      krs.setPos(1, j);
      delay(1);
    }
    for(int j=8500;j>6500;j=j-2){
      krs.setPos(1, j);
      delay(1);
    }
    for(int j=6500;j<7500;j=j+2){
      krs.setPos(1, j);
      delay(1);
    }
  }
  currentServoState = HOLD;
}

void init_arm_downward(){
  targetAngles[6] = 9500;
  totalTime[6] = 2.0;
  startTime[6] = millis() / 1000.0;  
  targetAngles[7] = 7500;
  totalTime[7] = 2.0;
  startTime[7] = millis() / 1000.0;
  targetAngles[8] = 7500;
  totalTime[8] = 2.0;
  startTime[8] = millis() / 1000.0;  
  targetAngles[9] = 7500;
  totalTime[9] = 2.0;
  startTime[9] = millis() / 1000.0;  
  targetAngles[10] = 4833;
  totalTime[10] = 2.0;
  startTime[10] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }
}

void grip_hold_arm(){
  targetAngles[7] = 9000;
  totalTime[7] = 2.0;
  startTime[7] = millis() / 1000.0;    
  targetAngles[9] = 10167;
  totalTime[9] = 2.0;
  startTime[9] = millis() / 1000.0;    
  targetAngles[10] = 6000;
  totalTime[10] = 2.0;
  startTime[10] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }  
}

void hooked_hold_arm(){
  targetAngles[9] = 8500;
  totalTime[9] = 2.0;
  startTime[9] = millis() / 1000.0;    
  targetAngles[10] = 6000;
  totalTime[10] = 2.0;
  startTime[10] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }  
}

void interlocked_fingers_arm(){
  targetAngles[6] = 5000;
  totalTime[6] = 2.0;
  startTime[6] = millis() / 1000.0;  
  targetAngles[10] = 6000;
  totalTime[10] = 2.0;
  startTime[10] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }  
}

void toddler_hold_arm(){
  targetAngles[3] = -10;
  totalTime[3] = 2.0;
  startTime[3] = millis() / 1000.0;
  targetAngles[7] = 9000;
  totalTime[7] = 2.0;
  startTime[7] = millis() / 1000.0;    
  targetAngles[9] = 10167;
  totalTime[9] = 2.0;
  startTime[9] = millis() / 1000.0;    
  targetAngles[10] = 6000;
  totalTime[10] = 2.0;
  startTime[10] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }  
}

void init_to_hooked(){
  targetAngles[1] = -30;
  totalTime[1] = 2.0;
  startTime[1] = millis() / 1000.0;
  targetAngles[2] = 35;
  totalTime[2] = 2.0;
  startTime[2] = millis() / 1000.0;
  targetAngles[4] = -35;
  totalTime[4] = 2.0;
  startTime[4] = millis() / 1000.0;
  targetAngles[3] = 30;
  totalTime[3] = 2.0;
  startTime[3] = millis() / 1000.0;

  targetAngles[0] = -30;
  totalTime[0] = 2.0;
  startTime[0] = millis() / 1000.0;
  targetAngles[5] = 30;
  totalTime[5] = 2.0;
  startTime[5] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }
  M5.Lcd.println("test end");
}

void hooked_to_init(){
  targetAngles[1] = 60;
  totalTime[1] = 2.0;
  startTime[1] = millis() / 1000.0;
  targetAngles[2] = -60;
  totalTime[2] = 2.0;
  startTime[2] = millis() / 1000.0;
  targetAngles[4] = 60;
  totalTime[4] = 2.0;
  startTime[4] = millis() / 1000.0;
  targetAngles[3] = -60;
  totalTime[3] = 2.0;
  startTime[3] = millis() / 1000.0;

  targetAngles[0] = -60;
  totalTime[0] = 2.0;
  startTime[0] = millis() / 1000.0;
  targetAngles[5] = -60;
  totalTime[5] = 2.0;
  startTime[5] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }
  M5.Lcd.println("test end");  
}

void init_to_grip(){
  targetAngles[1] = -30;
  totalTime[1] = 2.0;
  startTime[1] = millis() / 1000.0;
  targetAngles[2] = 35;
  totalTime[2] = 2.0;
  startTime[2] = millis() / 1000.0;
  targetAngles[4] = -35;
  totalTime[4] = 2.0;
  startTime[4] = millis() / 1000.0;
  targetAngles[3] = 30;
  totalTime[3] = 2.0;
  startTime[3] = millis() / 1000.0;

  targetAngles[0] = 5;
  totalTime[0] = 2.0;
  startTime[0] = millis() / 1000.0;
  targetAngles[5] = 20;
  totalTime[5] = 2.0;
  startTime[5] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }
  M5.Lcd.println("test end");
}

void grip_to_init(){
  targetAngles[1] = 60;
  totalTime[1] = 2.0;
  startTime[1] = millis() / 1000.0;
  targetAngles[2] = -60;
  totalTime[2] = 2.0;
  startTime[2] = millis() / 1000.0;
  targetAngles[4] = 60;
  totalTime[4] = 2.0;
  startTime[4] = millis() / 1000.0;
  targetAngles[3] = -60;
  totalTime[3] = 2.0;
  startTime[3] = millis() / 1000.0;

  targetAngles[0] = -60;
  totalTime[0] = 2.0;
  startTime[0] = millis() / 1000.0;
  targetAngles[5] = -60;
  totalTime[5] = 2.0;
  startTime[5] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }
  M5.Lcd.println("test end");  
}

void init_to_interlocked(){
  targetAngles[6] = 4600;
  totalTime[6] = 2.0;
  startTime[6] = millis() / 1000.0;  
  targetAngles[1] = -30;
  totalTime[1] = 3.0;
  startTime[1] = millis() / 1000.0;
  targetAngles[2] = 35;
  totalTime[2] = 3.0;
  startTime[2] = millis() / 1000.0;
  targetAngles[4] = -35;
  totalTime[4] = .0;
  startTime[4] = millis() / 1000.0;
  targetAngles[3] = 30;
  totalTime[3] = 3.0;
  startTime[3] = millis() / 1000.0;

  targetAngles[0] = -30;
  totalTime[0] = 3.0;
  startTime[0] = millis() / 1000.0;
  targetAngles[5] = 30;
  totalTime[5] = 3.0;
  startTime[5] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }
}

void interlocked_to_init(){
  targetAngles[1] = 60;
  totalTime[1] = 2.0;
  startTime[1] = millis() / 1000.0;
  targetAngles[2] = -60;
  totalTime[2] = 2.0;
  startTime[2] = millis() / 1000.0;
  targetAngles[4] = 60;
  totalTime[4] = 2.0;
  startTime[4] = millis() / 1000.0;
  targetAngles[3] = -60;
  totalTime[3] = 2.0;
  startTime[3] = millis() / 1000.0;

  targetAngles[0] = -60;
  totalTime[0] = 2.0;
  startTime[0] = millis() / 1000.0;
  targetAngles[5] = -60;
  totalTime[5] = 2.0;
  startTime[5] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }
}

void init_to_toddler(){
  targetAngles[6] = 4600;
  totalTime[6] = 2.0;
  startTime[6] = millis() / 1000.0;  
  targetAngles[2] = 0;
  totalTime[2] = 3.0;
  startTime[2] = millis() / 1000.0;
  targetAngles[4] = 0;
  totalTime[4] = 3.0;
  startTime[4] = millis() / 1000.0;

  targetAngles[0] = -30;
  totalTime[0] = 3.0;
  startTime[0] = millis() / 1000.0;
  targetAngles[5] = 5;
  totalTime[5] = 3.0;
  startTime[5] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }
  M5.Lcd.println("test end");
}

void toddler_to_init(){
  targetAngles[1] = 60;
  totalTime[1] = 2.0;
  startTime[1] = millis() / 1000.0;
  targetAngles[2] = -60;
  totalTime[2] = 2.0;
  startTime[2] = millis() / 1000.0;
  targetAngles[4] = 60;
  totalTime[4] = 2.0;
  startTime[4] = millis() / 1000.0;
  targetAngles[3] = -60;
  totalTime[3] = 2.0;
  startTime[3] = millis() / 1000.0;

  targetAngles[0] = -60;
  totalTime[0] = 2.0;
  startTime[0] = millis() / 1000.0;
  targetAngles[5] = -60;
  totalTime[5] = 2.0;
  startTime[5] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }
}

void hooked_hand_demo(){
  init_arm_downward();
  delay(1000);
  hooked_hold_arm();
  delay(1200);
  init_to_hooked();
  delay(1200);
  hooked_to_init();
  delay(500);
  init_arm_downward();
}

void grip_hand_demo(){
  init_arm_downward();
  delay(1000);
  grip_hold_arm();
  delay(1200);
  init_to_grip();
  delay(1200);
  grip_to_init();
  delay(500);
  init_arm_downward();
}

void interlocked_fingers_demo(){
  init_arm_downward();
  delay(1000);
  interlocked_fingers_arm();
  delay(1200);
  init_to_interlocked();
  delay(1200);
  interlocked_to_init();
  delay(500);
  init_arm_downward();
}

void toddler_hand_demo(){
  init_arm_downward();
  delay(1000);
  toddler_hold_arm();
  delay(1200);
  init_to_toddler();
  delay(1200);
  toddler_to_init();
  delay(300);
  init_arm_downward();
}

void read_servo(){
  for (int i = 1; i < 5; ++i) {
    int pos = krs.getPos(i);
    // USBSerial.println(pos);
  }
}

void test(){
  targetAngles[7] = 9591;
  totalTime[7] = 2.0;
  startTime[7] = millis() / 1000.0;  
  targetAngles[8] = 7456;
  totalTime[8] = 2.0;
  startTime[8] = millis() / 1000.0;  
  targetAngles[9] = 4833;
  totalTime[9] = 2.0;
  startTime[9] = millis() / 1000.0;  
  targetAngles[10] = 7059;
  totalTime[10] = 2.0;
  startTime[10] = millis() / 1000.0;
  updateCompleteFlag = false;
  while (true){
    if(updateCompleteFlag)
      break;
    else
      delay(5);
  }
  M5.Lcd.println("test end");
}

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


void smoothUpdate(void *parameter) {
  // static float velocity[NUM_SERVOS] = {0}; // 各サーボの速度
  // static float acceleration[NUM_SERVOS] = {0}; // 各サーボの加速度
  while(true){
    if(updateCompleteFlag){
       delay(1);
      continue;
    }
    bool allServosUpdated = true; // 全てのサーボの更新が完了したか確認するためのフラグ

    for (int i = 0; i < 6; i++) {
      if(abs(targetAngles[i] - currentAngles[i]) < 1){
        delay(1);
        continue;
      }
      float t = (millis() / 1000.0) - startTime[i]; // 経過時間 (秒)
      if (t >= totalTime[i]) {
        // 目標に到達した場合は、最終角度に設定
        currentAngles[i] = targetAngles[i];
        sbus.setServoAngle(i, currentAngles[i]);
      } else {
        float T = totalTime[i];
        float tau = t / T;
        float tau3 = tau * tau * tau;
        float tau4 = tau3 * tau;
        float tau5 = tau4 * tau;
        currentAngles[i] = currentAngles[i] + (targetAngles[i] - currentAngles[i]) *
          (10 * tau3 - 15 * tau4 + 6 * tau5);
        sbus.setServoAngle(i, currentAngles[i]);
        allServosUpdated = false;
        M5.Lcd.clear();
        M5.Lcd.setCursor(0,0);
        M5.Lcd.print(currentAngles[i]);
      }
    }
    if (!allServosUpdated) {
      sbus.sendSbusData();
      delay(3);
    }
    for (int i = 6; i < NUM_SERVOS; i++) {
      if(abs(targetAngles[i] - currentAngles[i]) < 1){
        delay(1);
        continue;
      }
      float t = (millis() / 1000.0) - startTime[i]; // 経過時間 (秒)
      if (t >= totalTime[i]) {
        // 目標に到達した場合は、最終角度に設定
        currentAngles[i] = targetAngles[i];
        krs.setPos(i-6, currentAngles[i]);
      } else {
        float T = totalTime[i];
        float tau = t / T;
        float tau3 = tau * tau * tau;
        float tau4 = tau3 * tau;
        float tau5 = tau4 * tau;
        currentAngles[i] = currentAngles[i] + (targetAngles[i] - currentAngles[i]) *
          (10 * tau3 - 15 * tau4 + 6 * tau5);
        M5.Lcd.clear();
        M5.Lcd.setCursor(0,0);
        M5.Lcd.print(currentAngles[i]);
        krs.setPos(i-6, currentAngles[i]);
        allServosUpdated = false;
        // USBSerial.println(currentAngles[i]);
      }
    }
    // USBSerial.print(" ");
    // USBSerial.println(targetAngles[i]);
    // 全てのサーボの更新が完了した場合
    if (allServosUpdated) {
      updateCompleteFlag = true; // 更新完了フラグを立てる
    }
    delay(2);
  }
}

void setup()
{
  // USBSerial.begin();
  // === ATOMS3 === //
  M5.begin();
  M5.Lcd.setRotation(3);  // 画面向き設定（USB位置基準 0：下/ 1：右/ 2：上/ 3：左）
  M5.Lcd.setTextSize(2);  // 文字サイズ（整数倍率）
  M5.Lcd.print("ICS Servo Controll");

  delay(500);

  // === Servo === //
  // ICS : データ長8bit, パリティEVEN, ストップビット1, 極性反転なし
  Serial1.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN, false, TIMEOUT);
  krs.begin();
  for(int i=1;i<5;i++){
    krs.setSpd(i,80);
  }
  initialize_servo();
  xTaskCreatePinnedToCore(smoothUpdate, "Smooth Update", 2048, NULL, 20, NULL, 0);
  xTaskCreatePinnedToCore(ButtonTask, "Button Task", 2048, NULL, 24, NULL, 1);
}


void loop()
{
  M5.update();  //本体ボタン状態更新

  // M5.Lcd.clear();
  // M5.Lcd.setCursor(0, 0);
  std::vector<int> ids;
  for (int i = 0; i < 18; ++i) {
    int pos = krs.getPos(i);
    // M5.Lcd.print(pos);
    if (pos != -1) {
      ids.push_back(i);
    }
  }
  // M5.Lcd.clear();
  // M5.Lcd.setCursor(0, 0);
  // for (size_t i = 0; i < ids.size(); ++i) {
  //   M5.Lcd.print(ids[i]);
  //   M5.Lcd.print(' ');
  // }

  if(currentButtonState==PRESSED){
    if(currentServoState==FREE){
      hold_arm();
    }else if(currentServoState==HOLD){
      free_arm();
    }
  }else if(currentButtonState==SINGLE_CLICK){
    M5.Lcd.clear();
    M5.Lcd.setCursor(0,0);
    M5.Lcd.println("start");
    delay(5000);
    hooked_hand_demo();
    delay(5000);
    grip_hand_demo();
    delay(5000);
    interlocked_fingers_demo();
    delay(5000);
    toddler_hand_demo();
  }else if(currentButtonState==DOUBLE_CLICK){

    // grip_to_init();
  }
  currentButtonState = NOT_CHANGED;
  delay(100);
}
