#include "CapacitiveSensorPalm.h"

CapacitiveSensorPalm::CapacitiveSensorPalm(uint8_t finger, uint8_t channel)
  : tla(I2C1_SDA, I2C1_SCL), rs()
{
  finger_num = finger;
  channel_num = channel;
  error = 1;
  loopTimingFactor = 310;		// determined empirically -  a hack
  
  CS_Timeout_Millis = (2000 * (float)loopTimingFactor * (float)F_CPU) / 16000000;
  CS_AutocaL_Millis = 20000;
  
  rs.setTXpinMode(finger_num, OUTPUT);
  rs.write(finger_num, LOW);

  // get pin mapping and port for receive Pin - from digital pin functions in Wiring.c
  leastTotal = 0x0FFFFFFFL;   // input large value for autocalibrate begin
  lastCal = millis();         // set millis for start
}

long CapacitiveSensorPalm::capacitiveSensor(uint8_t samples)
{
    rs.switchChannel(finger_num, channel_num/2); //2つずつ同じ
	total = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;            // bad pin


	for (uint8_t i = 0; i < samples; i++) {    // loop for samples parameter - simple lowpass filter
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
}

		// only calibrate if time is greater than CS_AutocaL_Millis and total is less than 10% of baseline
		// this is an attempt to keep from calibrating when the sensor is seeing a "touched" signal
		unsigned long diff = (total > leastTotal) ? total - leastTotal : leastTotal - total;
		if ( (millis() - lastCal > CS_AutocaL_Millis) && diff < (int)(.10 * (float)leastTotal) ) {
			leastTotal = 0x0FFFFFFFL;          // reset for "autocalibrate"
			lastCal = millis();
		}
        
	// routine to subtract baseline (non-sensed capacitance) from sensor return
	if (total < leastTotal) leastTotal = total;                 // set floor value to subtract from sensed value
	return(total - leastTotal);

}

long CapacitiveSensorPalm::capacitiveSensorRaw(uint8_t samples)
{
	total = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;                  // bad pin - this appears not to work

	for (uint8_t i = 0; i < samples; i++) {    // loop for samples parameter - simple lowpass filter
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
	}

	return total;
}

void CapacitiveSensorPalm::reset_CS_AutoCal(void){
	leastTotal = 0x0FFFFFFFL;
}

void CapacitiveSensorPalm::set_CS_AutocaL_Millis(unsigned long autoCal_millis){
	CS_AutocaL_Millis = autoCal_millis;
}

void CapacitiveSensorPalm::set_CS_Timeout_Millis(unsigned long timeout_millis){
	CS_Timeout_Millis = (timeout_millis * (float)loopTimingFactor * (float)F_CPU) / 16000000;  // floats to deal with large numbers
}

uint8_t CapacitiveSensorPalm::digitalReadI2C(){
  tla.selectADCChannel(finger_num, channel_num);
  int receive_value = tla.readI2C(finger_num);
  // return (receive_value > 4095 * 2.0 / 3.3) ? HIGH : LOW;
  return receive_value;
}

int CapacitiveSensorPalm::SenseOneCycle(void)
{
  // rs.switchChannel(finger_num, channel_num/2); //2つずつ同じ
  noInterrupts();
  rs.write(finger_num, LOW);
  delayMicroseconds(10);
  rs.write(finger_num, HIGH);
  interrupts();

  while ( !digitalReadI2C() && (total < CS_Timeout_Millis) ) {  // while receive pin is LOW AND total is positive value
    total++;
  }

  if (total > CS_Timeout_Millis) {
    return -2;         //  total variable over timeout
  }

	// set receive pin HIGH briefly to charge up fully - because the while loop above will exit when pin is ~ 2.5V
  noInterrupts();
  rs.write(finger_num, LOW);
  interrupts();

// #ifdef FIVE_VOLT_TOLERANCE_WORKAROUND
//     pinMode(rPin, OUTPUT);
//     digitalWrite(rPin, LOW);
// 	delayMicroseconds(10);
//     pinMode(rPin, INPUT);
// #else
  while ( digitalReadI2C() && (total < CS_Timeout_Millis) ) {  // while receive pin is HIGH  AND total is less than timeout
    total++;
  }
// #endif
  
  if (total >= CS_Timeout_Millis) {
    return -2;     // total variable over timeout
  } else {
    return 1;
  }
}

