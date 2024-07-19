#ifndef __SBUS_h__
#define __SBUS_h__
#include <M5AtomS3.h>

class SBUS
{
 public:
  HardwareSerial *futabaSerial;  ///<arudinoのシリアル型のポインタを格納
  int txPin, rxPin;
  long baudRate;
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
  SBUS(HardwareSerial *Serial, int RX_PIN, int TX_PIN, long baudrate);
  bool begin();
  void sendSbusData(void);
  void setServoAngle(int id, float angle);

};

#endif
