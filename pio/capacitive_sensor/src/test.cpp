#include "CapacitiveSensorPalm.h"
#include "RS2255.h"

CapacitiveSensorPalm cs_2_2 = CapacitiveSensorPalm(4,2);
// RS2255 rs = RS2255();

void setup(){
  Wire.begin(1,2);
  USBSerial.begin();
  cs_2_2.set_CS_AutocaL_Millis(0xFFFFFFFF);
  // rs.setTXpinMode(4, OUTPUT);
  // rs.switchChannel(4, 1); //2つずつ同じ
}

void loop(){
  long total1 =  cs_2_2.capacitiveSensor(30);
  // long total1 =  cs_2_2.digitalReadI2C();
  // rs.write(4, HIGH);
  // USBSerial.println("high");
  // delay(1000);
  // rs.write(4, LOW);
  // USBSerial.println("low");
  // delay(1000);
  USBSerial.print(total1);                  // print sensor output 1
  USBSerial.println("\t");
  delay(10); 
}
