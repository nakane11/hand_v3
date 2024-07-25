#ifndef WIFI_HARDWARE_H_
#define WIFI_HARDWARE_H_

#include <Arduino.h>
#include <WiFi.h>

IPAddress server(192,168,97,190);
uint16_t serverPort = 11411;

class WiFiHardware
{
  public:
  WiFiHardware(){}
  void init() { client.connect(server, serverPort); }
    int read() { return client.read(); }
    void write(uint8_t *data, int length) { for (int i = 0; i < length; i++) client.write(data[i]); }
    unsigned long time() { return millis(); }

 protected:
    WiFiClient client;
};

#endif

