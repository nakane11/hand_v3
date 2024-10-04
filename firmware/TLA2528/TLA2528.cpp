#include "TLA2528.h"

TLA2528::TLA2528(uint8_t sda, uint8_t scl)
{
  Wire.begin(sda, scl);
}

void TLA2528::selectADCChannel(uint8_t slave_addr, uint8_t channel) {
  Wire.beginTransmission(slave_addr);
  Wire.write(0x08); //single register write
  Wire.write(MANUAL_CH_SEL_ADDRESS);
  Wire.write(channel);
  Wire.endTransmission();
}

uint16_t TLA2528::readI2C(uint8_t slave_addr, uint8_t channel) {
  // selectADCChannel(slave_addr, channel);
  Wire.requestFrom(slave_addr, 2);
  uint16_t data = Wire.read() << 8 | Wire.read();
  return data;
}
