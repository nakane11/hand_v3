#ifndef CapacitiveSensorPalm_h
#define CapacitiveSensorPalm_h

#include <Arduino.h>
#include "TLA2528.h"
#include "RS2255.h"

#define I2C1_SCL 2
#define I2C1_SDA 1

class CapacitiveSensorPalm
{
  // user-accessible "public" interface
 public:
  // methods
  CapacitiveSensorPalm(uint8_t finger, uint8_t channel);
  long capacitiveSensorRaw(uint8_t samples);
  long capacitiveSensor(uint8_t samples);
  void set_CS_Timeout_Millis(unsigned long timeout_millis);
  void reset_CS_AutoCal();
  void set_CS_AutocaL_Millis(unsigned long autoCal_millis);
  uint8_t digitalReadI2C();
  // library-accessible "private" interface
 private:
  // variables
  int error;
  unsigned long  leastTotal;
  unsigned int   loopTimingFactor;
  unsigned long  CS_Timeout_Millis;
  unsigned long  CS_AutocaL_Millis;
  unsigned long  lastCal;
  unsigned long  total;
  uint8_t finger_num;
  uint8_t channel_num;
  TLA2528 tla;
  RS2255 rs;
  // methods
  int SenseOneCycle(void);
};

#endif
