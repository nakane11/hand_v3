#ifndef TLA2528_h
#define TLA2528_h
#include <Wire.h>

#define TLA2528_ADDR_A (0x16)
#define TLA2528_ADDR_B (0x15)
#define TLA2528_ADDR_C (0x10)
#define TLA2528_ADDR_D (0x11)
#define TLA2528_ADDR_E (0x12)

#define MANUAL_CH_SEL_ADDRESS (0x11)
#define MANUAL_CHID_AIN0                        ((uint8_t) 0x00)    // DEFAULT
#define MANUAL_CHID_AIN1                        ((uint8_t) 0x01)
#define MANUAL_CHID_AIN2                        ((uint8_t) 0x02)
#define MANUAL_CHID_AIN3                        ((uint8_t) 0x03)
#define MANUAL_CHID_AIN4                        ((uint8_t) 0x04)
#define MANUAL_CHID_AIN5                        ((uint8_t) 0x05)
#define MANUAL_CHID_AIN6                        ((uint8_t) 0x06)
#define MANUAL_CHID_AIN7                        ((uint8_t) 0x07)

class TLA2528
{
public:
  TLA2528(uint8_t sda, uint8_t scl);
  uint16_t readI2C(uint8_t slave_addr);
  void selectADCChannel(uint8_t slave_addr, uint8_t channel);
};

#endif
