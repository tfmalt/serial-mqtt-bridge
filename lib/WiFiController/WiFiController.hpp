/**
 * Class for managing the wifi connection
 */

#ifndef WiFiController_h
#define WiFiController_h

#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#elif ESP8266
#include <ESP8266WiFi.h>
#endif
// #include <LedshelfConfig.hpp>

class WiFiController {
public:
  WiFiController();

  void connect(const char *ssid, const char *psk, const char *hostname);

  WiFiClient &getWiFiClient();
  WiFiController &enableVerboseOutput();

private:
  bool VERBOSE = false;
  // WiFiClientSecure wifiClient;
  WiFiClient wifiClient;
#ifdef ESP32
  void handleEvent(WiFiEvent_t event);
#endif
};

#endif // WiFiController_h