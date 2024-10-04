#ifndef RS2255_h
#define RS2255_h

#include <Arduino.h> 

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

const uint8_t RS2255_S0[] = {RS2255_A_S0, RS2255_B_S0, RS2255_C_S0, RS2255_D_S0, RS2255_E_S0};
const uint8_t RS2255_S1[] = {RS2255_A_S1, RS2255_B_S1, RS2255_C_S1, RS2255_D_S1, RS2255_E_S1};
const uint8_t TX_PIN[] = {TX_A, TX_B, TX_C, TX_D, TX_E};

class RS2255 {
  public:
    RS2255();  // コンストラクタ宣言
    void switchChannel(uint8_t device, uint8_t channel);  // チャンネルを切り替える関数
    void write(uint8_t device, uint8_t val);  // 書き込みを行う関数
    void setTXpinMode(uint8_t device, uint8_t mode);  // TXピンのモードを設定する関数
};

#endif
