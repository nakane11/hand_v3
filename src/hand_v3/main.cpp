#include <M5AtomS3.h>
#include <IcsHardSerialClass.h>
#include <SBUS.h>

// === Servo === //
const byte EN_PIN = 6;
const byte RX_PIN = 5;
const byte TX_PIN = 38;
const long BAUDRATE = 1250000;
const int TIMEOUT = 20;
const byte KRS_ID = 6;
IcsHardSerialClass krs(&Serial1, EN_PIN, BAUDRATE, TIMEOUT);

// === Futaba === //
const byte RX_PIN_FUTABA = 10;
const byte TX_PIN_FUTABA = 39;
const long BAUDRATE_FUTABA = 100000;
SBUS sbus(&Serial2, RX_PIN_FUTABA, TX_PIN_FUTABA, BAUDRATE_FUTABA);

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
  // === Futaba === //
  sbus.begin();  
}

int prevId = -1;


void loop()
{
  M5.Lcd.print("3");
  M5.update();  //本体ボタン状態更新

  int reId = krs.getID();
  if (prevId != reId) {
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
    char log_msg[50];
    sprintf(log_msg, "ID: %d", reId);
    M5.Lcd.println(log_msg);
    prevId = reId;
  }

    // M5.Lcd.clear();
    // M5.Lcd.setCursor(0, 0);
    std::vector<int> ids;
    for (int i = 0; i < 18; ++i) {
      int pos = krs.getPos(i);
      // M5.Lcd.print(pos);
      // M5.Lcd.print(' ' );
      if (pos != -1) {
        ids.push_back(i);
      }
    }
    // char log_msg[50];
    // sprintf(log_msg, "ID: %d", reId);
      // M5.Lcd.println(log_msg);
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print('t');
    M5.Lcd.print(' ');    
    for (size_t i = 0; i < ids.size(); ++i) {
      M5.Lcd.print(ids[i]);
      M5.Lcd.print(' ');
    }

    for (int i=9500;i>=5500;i-=10){
      krs.setPos(0,i);
      delay(3);
    }
    for (int i=5500;i<=9500;i+=10){
      krs.setPos(0,i);
      delay(3);
    }
  
    float pos = 0;
  for (pos = 60; pos >= -30; pos -= 1) { 
    sbus.setServoAngle(1,pos);
    sbus.sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }

// little finger
  for (pos = 60; pos >= -30; pos -= 1) { 
    sbus.setServoAngle(1,pos);
    sbus.sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = -30; pos <= 60; pos += 1) {
    sbus.setServoAngle(1,pos);
    sbus.sendSbusData();             // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  
  // ring finger
  for (pos = -60; pos <= 40; pos += 1) { 
    sbus.setServoAngle(2,pos);
    sbus.sendSbusData();             // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 40; pos >= -60; pos -= 1) {
    sbus.setServoAngle(2,pos);
    sbus.sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }

  // middle finger
  for (pos = 60; pos >= -30; pos -= 1) {
    sbus.setServoAngle(4,pos);
    sbus.sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = -30; pos <= 60; pos += 1) {
    sbus.setServoAngle(4,pos);
    sbus.sendSbusData();             // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  
  // index finger
  for (pos = -60; pos <= 40; pos += 1) { 
    sbus.setServoAngle(3,pos);
    sbus.sendSbusData();             // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 40; pos >= -60; pos -= 1) {
    sbus.setServoAngle(3,pos);
    sbus.sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }

  //thumb
  for (pos = -60; pos <= 50; pos += 1) {
    sbus.setServoAngle(0,pos);
    sbus.sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 50; pos >= -60; pos -= 1) { 
    sbus.setServoAngle(0,pos);
    sbus.sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = -60; pos <= 60; pos += 1) { 
    sbus.setServoAngle(5,pos);
    sbus.sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 60; pos >= -60; pos -= 1) { 
    sbus.setServoAngle(5,pos);
    sbus.sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }    
  // M5.Lcd.clear();
  // M5.Lcd.setCursor(0, 0);
  // std::vector<int> servo_ids = krs.search_servo_ids();
  // char log_msg[50];
  // // sprintf(log_msg, "N: %d", servo_ids.size());
  // sprintf(log_msg, "N: %d", 10);
  // M5.Lcd.println(log_msg);

  // delay(100);

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
