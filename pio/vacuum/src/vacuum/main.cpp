#include <M5AtomS3.h>
#include <IcsHardSerialClass.h>
#include <VacuumEsp.h>

// === ICS Servo === //
const byte EN_PIN = 6;
const byte RX_PIN = 5;
const byte TX_PIN = 38;
const long BAUDRATE = 1250000;
const int TIMEOUT = 20;
IcsHardSerialClass krs(&Serial1, EN_PIN, BAUDRATE, TIMEOUT);
VacuumEsp vacuum(krs);
uint8_t sc;

void setup()
{
  M5.Lcd.begin();
  Serial1.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN, false, TIMEOUT);
  krs.begin();
  xTaskCreatePinnedToCore(VacuumEsp::pressureControlLoopWrapper, "Pressure control loop", 2048, &vacuum, 10, NULL, 0);
}

void loop()
{
  M5.Lcd.clear();
  M5.Lcd.setCursor(0,0);
  vacuum.updatePressure();
  M5.Lcd.print(vacuum.averagePressure());
  delay(100);
  // vacuum.start_vacuum();
  // delay(1000);
  // vacuum.stop_vacuum();
  // delay(1000);
  // vacuum.release_vacuum();
  // delay(1000);
}
