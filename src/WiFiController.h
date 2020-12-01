/**
 * Class for managing the wifi connection
 */

#ifndef WiFiController_h
#define WiFiController_h

#include <Arduino.h>
#include <string>
#ifdef ESP32
#include <WiFi.h>
#elif ESP8266
#include <ESP8266WiFi.h>
#endif

#define WIFI_DEFAULT_HOSTNAME "serialmqttbridge"

struct WiFiConfig {
  std::string ssid;
  std::string psk;
  std::string hostname;
};

class WiFiController {
 public:
  WiFiController();
  WiFiController(const char* ssid, const char* psk, const char* hostname);

  void connect();
  void connect(const char* ssid, const char* psk, const char* hostname);

  WiFiClient& getWiFiClient();
  WiFiController& enableVerboseOutput();

 private:
  WiFiConfig config;
  bool VERBOSE = false;
  WiFiClient wifiClient;
#ifdef ESP32
  void handleEvent(WiFiEvent_t event);
#endif
};

#endif  // WiFiController_h