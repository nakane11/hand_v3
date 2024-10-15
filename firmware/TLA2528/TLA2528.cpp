#include "TLA2528.h"

uint8_t tla_addr_list[5] = {TLA2528_ADDR_A, TLA2528_ADDR_B, TLA2528_ADDR_C, TLA2528_ADDR_D, TLA2528_ADDR_E};
uint8_t tla_channel_list[8] = {MANUAL_CHID_AIN0, MANUAL_CHID_AIN1, MANUAL_CHID_AIN2, MANUAL_CHID_AIN3,
                       MANUAL_CHID_AIN4, MANUAL_CHID_AIN5, MANUAL_CHID_AIN6, MANUAL_CHID_AIN7};

TLA2528::TLA2528(uint8_t sda, uint8_t scl)
{
  // Wire.begin(sda, scl);
}

void TLA2528::selectADCChannel(uint8_t slave_num, uint8_t channel_num) {
  Wire.beginTransmission(tla_addr_list[slave_num]);
  Wire.write(0x08); //single register write
  Wire.write(MANUAL_CH_SEL_ADDRESS);
  Wire.write(tla_channel_list[channel_num]);
  Wire.endTransmission(false);
}

uint16_t TLA2528::readI2C(uint8_t slave_num) {
  Wire.requestFrom(tla_addr_list[slave_num], 2);
  uint16_t data = Wire.read() << 8 | Wire.read();
  return data;
}
