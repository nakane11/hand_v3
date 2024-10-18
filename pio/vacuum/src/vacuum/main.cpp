#include <M5AtomS3.h>
#include <IcsHardSerialClass.h>
#include <VacuumEsp.h>
#include <OneButton.h>

// ==== Button ==== //
constexpr int BUTTON_PIN = 41;
OneButton btn(BUTTON_PIN, true, false);
enum ButtonState {
  NOT_CHANGED,
  SINGLE_CLICK,
  DOUBLE_CLICK,
  TRIPLE_CLICK,
  QUADRUPLE_CLICK,  // 4 clicks
  QUINTUPLE_CLICK,  // 5 clicks
  SEXTUPLE_CLICK,   // 6 clicks
  SEPTUPLE_CLICK,   // 7 clicks
  OCTUPLE_CLICK,    // 8 clicks
  NONUPLE_CLICK,    // 9 clicks
  DECUPLE_CLICK,    // 10 clicks
  PRESSED,
  RELEASED,
  RESET,
  BUTTON_STATE_COUNT
};
ButtonState currentButtonState = RESET;

static void handleClick() {
  currentButtonState = SINGLE_CLICK;
}

static void handleDoubleClick() {
  currentButtonState = DOUBLE_CLICK;
}

static void handleMultiClick() {
  int n = btn.getNumberClicks();
  switch (n) {
  case 1: currentButtonState = SINGLE_CLICK; break;
  case 2: currentButtonState = DOUBLE_CLICK; break;
  case 3: currentButtonState = TRIPLE_CLICK; break;
  case 4: currentButtonState = QUADRUPLE_CLICK; break;
  case 5: currentButtonState = QUINTUPLE_CLICK; break;
  case 6: currentButtonState = SEXTUPLE_CLICK; break;
  case 7: currentButtonState = SEPTUPLE_CLICK; break;
  case 8: currentButtonState = OCTUPLE_CLICK; break;
  case 9: currentButtonState = NONUPLE_CLICK; break;
  case 10: currentButtonState = DECUPLE_CLICK; break;
  default: currentButtonState = DECUPLE_CLICK; // Handle case where n > 10
  }
}

static void handleLongPress() {
  currentButtonState = PRESSED;
}

static void handleLongPressEnd() {
  currentButtonState = RELEASED;
}

void ButtonTask(void *parameter) {
  btn.setClickMs(200);  // Timeout used to distinguish single clicks from double clicks. (msec)
  btn.attachClick(handleClick);
  btn.attachDoubleClick(handleDoubleClick);
  btn.attachMultiClick(handleMultiClick);
  btn.attachLongPressStart(handleLongPress);
  btn.attachLongPressStop(handleLongPressEnd);// Initialize last receive time

  while (true) {
    btn.tick();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

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
  xTaskCreatePinnedToCore(ButtonTask, "Button Task", 2048, NULL, 24, NULL, 1);
  M5.Lcd.clear();
  M5.Lcd.setCursor(0,0);
  M5.Lcd.println("initialized");
}

void loop()
{
  vacuum.updatePressure();
  M5.Lcd.setCursor(0,20);
  M5.Lcd.println(vacuum.averagePressure());
  if(currentButtonState==PRESSED){
    if(vacuum.pressure_control_running){
      M5.Lcd.setCursor(0,40);
      M5.Lcd.println("release");
      vacuum.release = true;
    }else{
      M5.Lcd.setCursor(0,40);
      M5.Lcd.println("vacuum");
      vacuum.pressure_control_running=true;
    }
  }
  currentButtonState = NOT_CHANGED;
  delay(50);
}
