#include <M5AtomS3.h>
#include <IcsHardSerialClass.h>

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

int pos = 0;
bool sent_data = false;

char sbus_data[25] = {
  0x0f, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00
};
short  sbus_servo_id[16];

void sendSbusData(void){
    /* sbus送信用に変換した目標角度をS.BUS送信用バッファに詰め込む  */
    sbus_data[0] = 0x0f;
    sbus_data[1] =  sbus_servo_id[0] & 0xff;
    sbus_data[2] = ((sbus_servo_id[0] >> 8) & 0x07 ) | ((sbus_servo_id[1] << 3 ) );
    sbus_data[3] = ((sbus_servo_id[1] >> 5) & 0x3f ) | (sbus_servo_id[2]  << 6);
    sbus_data[4] = ((sbus_servo_id[2] >> 2) & 0xff ) ;
    sbus_data[5] = ((sbus_servo_id[2] >> 10) & 0x01 ) | (sbus_servo_id[3] << 1 )   ;
    sbus_data[6] = ((sbus_servo_id[3] >> 7) & 0x0f ) | (sbus_servo_id[4]  << 4 )   ;
    sbus_data[7] = ((sbus_servo_id[4] >> 4) & 0x7f ) | (sbus_servo_id[5]  << 7 )   ;
    sbus_data[8] = ((sbus_servo_id[5] >> 1) & 0xff ) ;
    sbus_data[9] = ((sbus_servo_id[5] >> 9) & 0x03 ) ;

    /* sbus_dataを25bit分送信 */  
    Serial2.write(sbus_data, 25); 
}

void setServoAngle(int id, float angle) {
  if (id < 0 || id >= 16) return; // IDが範囲外の場合は何もしない
  sbus_servo_id[id] = (int)(3071.5+1023.5/60*angle);
}

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
  Serial2.begin(BAUDRATE_FUTABA, SERIAL_8E2, RX_PIN_FUTABA, TX_PIN_FUTABA, true, TIMEOUT);
  
}

int prevId = -1;


void loop()
{
  M5.update();  //本体ボタン状態更新

  // int reId = krs.getID();
  // if (prevId != reId) {
  //   M5.Lcd.clear();
  //   M5.Lcd.setCursor(0, 0);
  //   char log_msg[50];
  //   sprintf(log_msg, "ID: %d", reId);
  //   M5.Lcd.println(log_msg);
  //   prevId = reId;
  // }

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
    setServoAngle(1,pos);
    sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }

// little finger
  for (pos = 60; pos >= -30; pos -= 1) { 
    setServoAngle(1,pos);
    sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = -30; pos <= 60; pos += 1) {
    setServoAngle(1,pos);
    sendSbusData();             // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  
  // ring finger
  for (pos = -60; pos <= 40; pos += 1) { 
    setServoAngle(2,pos);
    sendSbusData();             // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 40; pos >= -60; pos -= 1) {
    setServoAngle(2,pos);
    sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }

  // middle finger
  for (pos = 60; pos >= -30; pos -= 1) {
    setServoAngle(4,pos);
    sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = -30; pos <= 60; pos += 1) {
    setServoAngle(4,pos);
    sendSbusData();             // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  
  // index finger
  for (pos = -60; pos <= 40; pos += 1) { 
    setServoAngle(3,pos);
    sendSbusData();             // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 40; pos >= -60; pos -= 1) {
    setServoAngle(3,pos);
    sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }

  //thumb
  for (pos = -60; pos <= 50; pos += 1) {
    setServoAngle(0,pos);
    sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 50; pos >= -60; pos -= 1) { 
    setServoAngle(0,pos);
    sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = -60; pos <= 60; pos += 1) { 
    setServoAngle(5,pos);
    sendSbusData();              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 60; pos >= -60; pos -= 1) { 
    setServoAngle(5,pos);
    sendSbusData();              // tell servo to go to position in variable 'pos'
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
