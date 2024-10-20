#include "VacuumEsp.h"

VacuumEsp::VacuumEsp(IcsHardSerialClass& icsinstance) : ics(icsinstance) {
  gpio_state = 0x90;
  pinMode(1,OUTPUT_OPEN_DRAIN);
  pinMode(2,OUTPUT_OPEN_DRAIN);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  pressure_control_running = 0;
  release = 0;
}

float VacuumEsp::readPressure(byte id)
{
  byte txCmd[2];
  byte rxCmd[44];
  unsigned int adc_raw;
  bool flg;
  float v_amplified, v_diff, pressure;

  txCmd[0] = 0xA0 + id;               // CMD
  txCmd[1] = 0x4F;

  //送受信
  flg = ics.synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);

  int index = 8; 
  adc_raw = ((0x1f & rxCmd[index]) << 7) | rxCmd[index + 1];  

  v_amplified = 3.3 * adc_raw / 4095;
  v_diff = (v_amplified - V_OFFSET) / GAIN;
  pressure = (v_diff * 1000 * 5.0 / 4.14 - 10.739) / 3.1395;
  return pressure;
}

void VacuumEsp::updatePressure() {
    float new_pressure = readPressure(19);
    recent_pressures[pressure_index] = new_pressure;
    pressure_index = (pressure_index + 1) % PRESSURE_HISTORY_SIZE;
}

float VacuumEsp::averagePressure() {
    float sum = 0.0;
    for (int i = 0; i < PRESSURE_HISTORY_SIZE; i++) {
        sum += recent_pressures[i];
    }
    return sum / PRESSURE_HISTORY_SIZE;
}

void VacuumEsp::sendGPIO(byte id, uint8_t sc)
{
  byte txCmd[2];
  byte rxCmd[2];
  bool flg;

  txCmd[0] = 0xA0 + id;               // CMD
  txCmd[1] = sc;

  //送受信
  flg = ics.synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  gpio_state = sc;
}

void VacuumEsp::start_pump()
{
  digitalWrite(1, HIGH);
}

void VacuumEsp::stop_pump()
{
  digitalWrite(1, LOW);
}

void VacuumEsp::open_air_connect_valve()
{
  digitalWrite(2, HIGH);
}

void VacuumEsp::close_air_connect_valve()
{
  digitalWrite(2, LOW);
}

void VacuumEsp::open_work_valve()
{
  uint8_t sc;
  sc = gpio_state | 0x01;
  sendGPIO(19, sc);
}

void VacuumEsp::close_work_valve()
{
  uint8_t sc;
  sc = gpio_state & 0xFE;
  sendGPIO(19, sc);
}

void VacuumEsp::release_vacuum()
{
  stop_pump();
  open_work_valve();
  open_air_connect_valve();
  delay(1000);
  close_air_connect_valve();
  close_work_valve();
}

void VacuumEsp::start_vacuum()
{
  start_pump();
  open_work_valve();
  close_air_connect_valve();
}

void VacuumEsp::stop_vacuum()
{
  close_work_valve();
  close_air_connect_valve();
  delay(300);
  stop_pump();
}

void VacuumEsp::pressureControlLoop()
{
  float start_pressure = -10.0;
  float stop_pressure = -30.0;

  while(true){
    updatePressure(); //気圧更新
    M5.Lcd.setCursor(0,20);
    M5.Lcd.println(averagePressure());
    if(release){
      release_vacuum();
      release = false;
      pressure_control_running = false;
      vacuum_on = false;
      continue;
    }
    if(pressure_control_running){
      if(!vacuum_on & averagePressure() > start_pressure){
          start_vacuum();
          vacuum_on = true;
        }
      if(vacuum_on & averagePressure() <= stop_pressure){
        stop_vacuum();
        vacuum_on = false;
      }
    }
    delay(100);
  }
}

