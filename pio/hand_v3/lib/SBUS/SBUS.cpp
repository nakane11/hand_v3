#include "SBUS.h"

SBUS::SBUS(HardwareSerial *Serial, int RX_PIN, int TX_PIN, long baudrate)
{
  futabaSerial = Serial;
  rxPin = RX_PIN;
  txPin = TX_PIN;
  baudRate = baudrate;
}

bool SBUS::begin()
{
  futabaSerial->begin(baudRate, SERIAL_8E2, rxPin, txPin, true, 20);
  return true;
}
void SBUS::sendSbusData(void){
    /* sbus送信用に変換した目標角度をS.BUS送信用バッファに詰め込む  */
    sbus_data[0] = 0x0f;
    sbus_data[1] =  sbus_servo_id[0] & 0xff;
    sbus_data[2] = ((sbus_servo_id[0] >> 8) & 0x07 ) | ((sbus_servo_id[1] << 3 ) );
    sbus_data[3] = ((sbus_servo_id[1] >> 5) & 0x3f ) | (sbus_servo_id[2]  << 6);
    sbus_data[4] = ((sbus_servo_id[2] >> 2) & 0xff ) ;
    sbus_data[5] = ((sbus_servo_id[2] >> 10) & 0x01 ) | (sbus_servo_id[3] << 1 )   ;
    sbus_data[6] = ((sbus_servo_id[3] >> 7) & 0x0f ) | (sbus_servo_id[4]  << 4 )   ;
    sbus_data[7] = ((sbus_servo_id[4] >> 4) & 0x7f ) | (sbus_servo_id[5]  << 7 )   ;
    sbus_data[8] = ((sbus_servo_id[5] >> 1) & 0xff ) ;
    sbus_data[9] = ((sbus_servo_id[5] >> 9) & 0x03 ) ;

    /* sbus_dataを25bit分送信 */  
    futabaSerial->write(sbus_data, 25); 
}

void SBUS::setServoAngle(int id, float angle) {
  if (id < 0 || id >= 16) return; // IDが範囲外の場合は何もしない
  sbus_servo_id[id] = (int)(3071.5+1023.5/60*angle);
}

  
