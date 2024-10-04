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
const byte KRS_ID = 6;
IcsHardSerialClass krs(&Serial1, EN_PIN, BAUDRATE, TIMEOUT);

// === FUTABA Servo === //
const byte RX_PIN_FUTABA = 10;
const byte TX_PIN_FUTABA = 39;
const long BAUDRATE_FUTABA = 100000;
SBUS sbus(&Serial2, RX_PIN_FUTABA, TX_PIN_FUTABA, BAUDRATE_FUTABA);

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

void init_to_open_hand(){
  for(int i=9500;i>4900;i=i-1){
    krs.setPos(0,i);
  }
}

void open_to_init_hand(){
  for(int i=4900;i<=9500;i=i+1){
    krs.setPos(0,i);
  }
}

// void init_to_rock_hand(){
  
// }

void setup()
{
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

  sbus.setServoAngle(1,60);
  sbus.setServoAngle(2,-60);
  sbus.setServoAngle(4,60);
  sbus.setServoAngle(3,-60);
  sbus.setServoAngle(0,-60);
  sbus.setServoAngle(5,-60);
  sbus.begin();

  xTaskCreatePinnedToCore(ButtonTask, "Button Task", 2048, NULL, 24, NULL, 1);
}

int prevId = -1;


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
  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  for (size_t i = 0; i < ids.size(); ++i) {
    M5.Lcd.print(ids[i]);
    M5.Lcd.print(' ');
  }

  if(currentButtonState==PRESSED){
    if(currentServoState==FREE){
      hold_arm();
    }else if(currentServoState==HOLD){
      free_arm();
    }
  }else if(currentButtonState==SINGLE_CLICK){
    reset_arm();
  }else if(currentButtonState==DOUBLE_CLICK){
    init_to_open_hand();
    wave_arm();
    open_to_init_hand();
  }
  currentButtonState = NOT_CHANGED;

  delay(100);

  // for (size_t i = 0; i < servo_ids.size(); ++i) {
  //   M5.Lcd.print(servo_ids[i]);
  //   M5.Lcd.print(' ');
  // }

  // // === Push   ：Rotates to 90deg === //
  // // === Release：Rotates to  0deg === //
  // if(M5.Btn.isPressed())
  // {
  //   if(pos != krs.degPos(90))
  //   {
  //     pos = krs.degPos(90);
  //     int s = krs.setPos(KRS_ID, pos);
  //     if(s == -1)
  //     {
  //       M5.Lcd.clear();
  //       M5.Lcd.setCursor(0, 0);
  //       M5.Lcd.print("Failed to send data.");
  //     }
  //     else
  //     {
  //       M5.Lcd.clear();
  //       M5.Lcd.setCursor(0, 0);
  //       M5.Lcd.print("Succeed to send data.");
  //     }
  //   }
  // }

  // else if(M5.Btn.isReleased())
  // {
  //   if(pos != krs.degPos(0))
  //   {
  //     pos = krs.degPos(0);
  //     int s = krs.setPos(KRS_ID, pos);
  //     if(s == -1)
  //     {
  //       M5.Lcd.clear();
  //       M5.Lcd.setCursor(0, 0);
  //       M5.Lcd.print("Failed to send data.");
  //     }
  //     else
  //     {
  //       M5.Lcd.clear();
  //       M5.Lcd.setCursor(0, 0);
  //       M5.Lcd.print("Succeed to send data.");
  //     }
  //   }
  // }
}
