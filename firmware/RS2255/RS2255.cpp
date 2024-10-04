#include "RS2255.h"

RS2255::RS2255(){
}

void RS2255::switchChannel(uint8_t device, uint8_t channel){
  if(device < 4 && channel < 4){
    digitalWrite(RS2255_S0[device], channel & 1);  // channelの最下位ビットをS0に送る
    digitalWrite(RS2255_S1[device], (channel >> 1) & 1);  // channelの2番目のビットをS1に送る
  }
}

void RS2255::write(uint8_t device, uint8_t val){
    digitalWrite(TX_PIN[device], val);
}

void RS2255::setTXpinMode(uint8_t device, uint8_t mode){
  pinMode(TX_PIN[device], mode);  
}
