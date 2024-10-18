#include <M5AtomS3.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <WiFi.h>
#include <WiFiHardware.h>
#include <ADS7828.h>

ADS7828 ads1 = ADS7828(0x48);
ADS7828 ads2 = ADS7828(0x49);
ADS7828 ads3 = ADS7828(0x4A);
ADS7828 ads4 = ADS7828(0x4B);
#define NCHANNELS 8

// === WiFi === //
const char SSID[] ="ist-hsr-2.4g";
const char PASSWORD[] = "89sk389sk3";

// === ROS === //
// Node Handler
ros::NodeHandle_<WiFiHardware> nh;
uint8_t force_data1[NCHANNELS];
uint8_t force_data2[NCHANNELS];
uint8_t force_data3[NCHANNELS];
uint8_t force_data4[NCHANNELS];
std_msgs::UInt8MultiArray force_msg;
ros::Publisher force_pub1("force_sensor1", &force_msg);
ros::Publisher force_pub2("force_sensor2", &force_msg);
ros::Publisher force_pub3("force_sensor3", &force_msg);
ros::Publisher force_pub4("force_sensor4", &force_msg);
byte adc_buff[2];
unsigned long loopTime;
#define LOOP_TIME 20

void setup()
{
  M5.begin();
  M5.Lcd.setRotation(3);  // 画面向き設定（USB位置基準 0：下/ 1：右/ 2：上/ 3：左）
  M5.Lcd.setTextSize(2);  // 文字サイズ（整数倍率）
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
  nh.advertise(force_pub1);
  nh.advertise(force_pub2);
  nh.advertise(force_pub3);
  nh.advertise(force_pub4);
  while(!nh.connected())
  {
    nh.spinOnce();
    delay(10);
  }
  Wire.begin(8,7);
  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("ROS Connected");
}

void loop()
{
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
  loopTime = millis();
  
  for(int j = 0; j < NCHANNELS; j++){
    force_data1[j] = ads1.getValue(j);
  }
  force_msg.data =  force_data1;
  force_msg.data_length = NCHANNELS;
  force_pub1.publish(&force_msg);
  for(int j = 0; j < NCHANNELS; j++){
    force_data2[j] = ads2.getValue(j);
  }
  force_msg.data =  force_data2;
  force_msg.data_length = NCHANNELS;
  force_pub2.publish(&force_msg);
  for(int j = 0; j < NCHANNELS; j++){
    force_data3[j] = ads3.getValue(j);
  }
  force_msg.data =  force_data3;
  force_msg.data_length = NCHANNELS;
  force_pub3.publish(&force_msg);
  for(int j = 0; j < NCHANNELS; j++){
    force_data4[j] = ads4.getValue(j);
  }
  force_msg.data =  force_data4;
  force_msg.data_length = NCHANNELS;
  force_pub4.publish(&force_msg);
  while (millis() < loopTime + LOOP_TIME); // enforce constant loop time
  nh.spinOnce();
}
