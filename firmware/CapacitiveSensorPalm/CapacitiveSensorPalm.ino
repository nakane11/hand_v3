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

#define RS2255_A_S0 4 
#define RS2255_A_S1 5
#define RS2255_B_S0 6 
#define RS2255_B_S1 7
#define RS2255_C_S0 17 
#define RS2255_C_S1 18
#define RS2255_D_S0 8 
#define RS2255_D_S1 9
#define RS2255_E_S0 10 
#define RS2255_E_S1 11

#define TX_A 12
#define TX_B 13
#define TX_C 14
#define TX_D 21
#define TX_E 47

#define GPIO_L 48

#define I2C1_SCL 38
#define I2C1_SDA 39

void selectADCChannel(byte slave_addr, byte channel) {
  Wire.beginTransmission(slave_addr);
  Wire.write(0x08); //single register write
  Wire.write(MANUAL_CH_SEL_ADDRESS);
  Wire.write(channel);
  Wire.endTransmission();
}

uint16_t readI2C(byte slave_addr, byte channel) {
  selectChannel(slave_addr, channel);
  Wire.requestFrom(slave_addr, 2);
  uint16_t data = Wire.read() << 8 | Wire.read();
  return data;
}

void setup() {
  Wire.begin(I2C1_SDA, I2C1_SCL);
}

void loop() {
  // put your main code here, to run repeatedly:

}
