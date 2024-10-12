#ifndef __vacuum_esp_h__
#define __vacuum_esp_h__
#include "Arduino.h"

#include <IcsHardSerialClass.h>

#define V_OFFSET 1.65
#define GAIN 7.4

#define PRESSURE_HISTORY_SIZE 20

class VacuumEsp
{
private:
  IcsHardSerialClass& ics;
  float recent_pressures[PRESSURE_HISTORY_SIZE] = {0};  // 直近10個のセンサ値を格納する配列
  int pressure_index = 0;
  uint8_t gpio_state;
  uint8_t vacuum_on = false;

  float readPressure(byte id);
  void sendGPIO(byte id, uint8_t sc);
  void start_pump();
  void stop_pump();
  void open_air_connect_valve();
  void close_air_connect_valve();
  void open_work_valve();
  void close_work_valve();
  void release_vacuum();
  void start_vacuum();
  void stop_vacuum();
  void pressureControlLoop();

public:

  uint8_t release = false;
  uint8_t pressure_control_running = false;
  
  VacuumEsp(IcsHardSerialClass& icsinstance);

  void updatePressure();
  float averagePressure();
  static void pressureControlLoopWrapper(void* pvParameters) {
    // pvParameters から VacuumEsp インスタンスを取得し、メンバ関数を呼び出す
    VacuumEsp* instance = static_cast<VacuumEsp*>(pvParameters);
    instance->pressureControlLoop();
  }  

};

#endif
