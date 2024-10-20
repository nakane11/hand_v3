#include <M5AtomS3.h>
#include <IcsHardSerialClass.h>
#include <SBUS.h>
#include <WiFi.h>
#include <WiFiHardware.h>
#include <ros.h>
#include <hand_v3/RawAngle.h>
#include <std_msgs/Int8MultiArray.h>

// === WiFi === //
const char SSID[] ="ist-hsr-2.4g";
const char PASSWORD[] = "89sk389sk3";

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

// === ROS === //
// Node Handler
ros::NodeHandle_<WiFiHardware> nh;

// Subscliber
void receiveFutabaCommandAngle(const hand_v3::RawAngle &msg)
{
  int length = msg.length;
  for (int i=0;i<length;i++){
    sbus.setServoAngle(msg.id[i], msg.angle[i]);
  }
  sbus.sendSbusData();
  delay(5);
}
void receiveKondoCommandAngle(const hand_v3::RawAngle &msg)
{
  int length = msg.length;
  for (int i=0;i<length;i++){
    krs.setPos(msg.id[i], msg.angle[i]);
    delay(1);
  }
}
void receiveKondoCommandSpeed(const hand_v3::RawAngle &msg)
{
  int length = msg.length;
  for (int i=0;i<length;i++){
    krs.setSpd(msg.id[i], msg.angle[i]);
    delay(3);
  }
}
ros::Subscriber<hand_v3::RawAngle> futaba_sub("futaba/command_angle", &receiveFutabaCommandAngle);
ros::Subscriber<hand_v3::RawAngle> kondo_angle_sub("kondo/command_angle", &receiveKondoCommandAngle);
ros::Subscriber<hand_v3::RawAngle> kondo_speed_sub("kondo/command_speed", &receiveKondoCommandSpeed);
std_msgs::Int8MultiArray id_msg;
ros::Publisher id_pub("kondo/servo_ids", &id_msg);

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
  sbus.setServoAngle(1,60);
  sbus.setServoAngle(2,-60);
  sbus.setServoAngle(4,60);
  sbus.setServoAngle(3,-60);
  sbus.setServoAngle(0,-60);
  sbus.setServoAngle(5,-60);
  sbus.begin();

   // === WiFi === //
  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("WiFi Connecting ...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
    delay(10);

  // === ROS === //
  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("ROS Connecting ...");
  do { M5.update(); delay(10); } while (!M5.BtnA.isPressed());
  nh.initNode();
  nh.subscribe(futaba_sub);
  nh.subscribe(kondo_angle_sub);
  nh.subscribe(kondo_speed_sub);
  nh.advertise(id_pub);
  while(!nh.connected())
  {
    nh.spinOnce();
    delay(10);
  }
  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("ROS Connected");
}

int prevId = -1;


void loop()
{
  M5.update();  //本体ボタン状態更新

  if (WiFi.status() != WL_CONNECTED)
  {
    nh.spinOnce();
    nh.logerror("No WiFi");
    delay(10);
    return;
  }

  // Check ROS connection.
  if (!nh.connected())
  {
    nh.spinOnce();
    nh.logwarn("No ROS");
    delay(10);
    return;
  }

  // sample_msg.data = 1;
  // sample_pub.publish(&sample_msg);

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
  id_msg.data_length =ids.size();
  id_msg.data = new int8_t[id_msg.data_length];
  
  // idsの要素をint8_tにキャストしてid_msg.dataにコピー
  for (size_t i = 0; i < ids.size(); ++i) {
    id_msg.data[i] = static_cast<int8_t>(ids[i]);
  }
  id_pub.publish(&id_msg);
  nh.spinOnce();
  
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print('t');
    M5.Lcd.print(' ');    
    for (size_t i = 0; i < ids.size(); ++i) {
      M5.Lcd.print(ids[i]);
      M5.Lcd.print(' ');
    }
}
