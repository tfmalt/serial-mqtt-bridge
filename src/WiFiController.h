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

#define WIFI_DEFAULT_HOSTNAME "serialmqttbridge"

class WiFiController {
 public:
  WiFiController();
  WiFiController(const char* ssid,
                 const char* psk,
                 const char* hostname = WIFI_DEFAULT_HOSTNAME)
      : _ssid(ssid), _psk(psk), _hostname(hostname){};

  void connect();
  void connect(const char* ssid, const char* psk, const char* hostname);

  WiFiClient& getWiFiClient();
  WiFiController& enableVerboseOutput();

 private:
  const char* _ssid;
  const char* _psk;
  const char* _hostname;

  bool VERBOSE = false;
  WiFiClient wifiClient;
#ifdef ESP32
  void handleEvent(WiFiEvent_t event);
#endif
};

#endif  // WiFiController_h