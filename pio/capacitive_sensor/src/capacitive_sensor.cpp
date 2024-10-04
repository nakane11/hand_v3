#include <Arduino.h>
#include "TLA2528.h"
#include "RS2255.h"

#define GPIO_L 48

#define I2C1_SCL 38
#define I2C1_SDA 39

TLA2528 tla(I2C1_SDA, I2C1_SCL);


void setup(){
}
void loop(){
}
