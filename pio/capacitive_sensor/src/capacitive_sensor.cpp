#include <Arduino.h>
#include "TLA2528.h"
#include "RS2255.h"

#define GPIO_L 48

#define I2C1_SCL 38
#define I2C1_SDA 39

TLA2528 tla(I2C1_SDA, I2C1_SCL);
RS2255 rs();

void setup(){
}
void loop(){
  for (int i=0; i<5; i++){ //指A~E
    for(int j=0;j<8;j++){
      tla.selectADCChannel(tla_addr_list[i], tla_channel_list[j]);
      rs.switchChannel(i, j/2); //2つずつ同じ
      rs.setTXpinMode(i, OUTPUT);
      rs.write(i, HIGH);
      data = tla.readI2C(tla_addr_list[i]);

}
