#ifndef WIFISERIAL_H
#define WIFISERIAL_H

#include <Arduino.h>

class WiFiSerial {
 private:
 public:
  WiFiSerial(){};

  setup() { Serial3.begin(962100, SERIAL_8N1); }
}

#endif  // WIFISERIAL_H