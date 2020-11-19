#include "WiFiController.hpp"
#include <Credentials.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

WiFiController::WiFiController() {}

/**
 * Configure and setup wifi
 */
void WiFiController::connect(const char* ssid,
                             const char* psk,
                             const char* hostname) {
  if (VERBOSE) {
    Serial.printf("[wifi] ||| connecting to ssid: %s: ", ssid);
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

#ifdef ESP32
  WiFi.onEvent(
      [this](WiFiEvent_t event, WiFiEventInfo_t info) { handleEvent(event); });
#elif ESP8266
  WiFi.onStationModeConnected([this](WiFiEventStationModeConnected event) {
    if (this->VERBOSE) {
      Serial.printf("[wifi] connected to: %s\n", event.ssid.c_str());
    }
  });

  WiFi.onStationModeDisconnected(
      [this](WiFiEventStationModeDisconnected event) {
        if (VERBOSE) {
          // WiFiDisconnectReason reason = event.reason;
          Serial.printf("[wifi] disconnected: %i\n", event.reason);
        }
      });

  WiFi.onStationModeDHCPTimeout([this]() {
    if (VERBOSE) {
      Serial.printf("[wifi] dhcp request timed out.\n");
    }
  });

  WiFi.onStationModeGotIP([this](WiFiEventStationModeGotIP event) {
    if (VERBOSE) {
      Serial.printf("[wifi]   ip address: %s\n", event.ip.toString().c_str());
      Serial.printf("[wifi]   gateway:    %s\n", event.gw.toString().c_str());
      Serial.printf("[wifi]   netmask:    %s\n", event.mask.toString().c_str());
    }
  });
#endif
  WiFi.begin(ssid, psk);

#ifdef ESP32
  WiFi.setHostname(MQTT_CLIENT);
#endif

  // Wait here until we are connected.
  while (WiFi.status() != WL_CONNECTED) {
    if (VERBOSE) {
      Serial.print(".");
    }
    delay(500);
  }

  if (VERBOSE) {
    Serial.println(" success.");
    IPAddress ip = WiFi.localIP();
    Serial.printf("[wifi] ||| ip address: %s\n", ip.toString().c_str());
  }
};

// WiFiClientSecure &WiFiController::getWiFiClient()
WiFiClient& WiFiController::getWiFiClient() {
  return wifiClient;
};

WiFiController& WiFiController::enableVerboseOutput() {
  VERBOSE = true;
  return *this;
}

#ifdef ESP32
void WiFiController::handleEvent(WiFiEvent_t event) {
  switch (event) {
    // SYSTEM_EVENT_WIFI_READY               < ESP32 WiFi ready
    case SYSTEM_EVENT_WIFI_READY:
#ifdef DEBUG
      Serial.printf("[wifi]   WiFi Ready [%i]\n", event);
#endif
      break;
    // SYSTEM_EVENT_SCAN_DONE                < ESP32 finish scanning AP
    case SYSTEM_EVENT_SCAN_DONE:
#ifdef DEBUG
      Serial.printf("[wifi]   got SYSTEM_EVENT_SCAN_DONE [%i]\n", event);
#endif
      break;
    // SYSTEM_EVENT_STA_START                < ESP32 station start
    case SYSTEM_EVENT_STA_START:
#ifdef DEBUG
      Serial.printf("[wifi]   Starting [%i] ...\n", event);
#endif
      break;
    // SYSTEM_EVENT_STA_STOP                 < ESP32 station stop
    case SYSTEM_EVENT_STA_STOP:
#ifdef DEBUG
      Serial.printf("[wifi]   got SYSTEM_EVENT_STA_STOP [%i]\n", event);
#endif
      break;
    // SYSTEM_EVENT_STA_CONNECTED            < ESP32 station connected to AP
    case SYSTEM_EVENT_STA_CONNECTED:
#ifdef DEBUG
      Serial.printf("[wifi]   Connected to access point \"%s\" [%i]\n",
                    WIFI_SSID, event);
#endif
      break;
    // SYSTEM_EVENT_STA_DISCONNECTED         < ESP32 station disconnected from
    // AP
    case SYSTEM_EVENT_STA_DISCONNECTED:
#ifdef DEBUG
      Serial.printf("[wifi]   got SYSTEM_EVENT_STA_DISCONNECTED [%i]\n", event);
      Serial.println("[wifi] restarting...");
#endif
      // sleeping for half a second to let things settle
      delay(500);
      ESP.restart();
      break;
    // SYSTEM_EVENT_STA_AUTHMODE_CHANGE      < the auth mode of AP connected by
    // ESP32 station changed
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
#ifdef DEBUG
      Serial.printf("[wifi]   got SYSTEM_EVENT_STA_AUTHMODE_CHANGE [%i]\n",
                    event);
#endif
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
#ifdef DEBUG
      Serial.printf("[wifi]   Got IP address [%i]\n", event);
      Serial.print("[wifi]      IP:  ");
      Serial.println(WiFi.localIP());
      Serial.print("[wifi]      DNS: ");
      Serial.println(WiFi.dnsIP());
#endif
      break;
    default:
#ifdef DEBUG
      Serial.printf("[wifi] Got other event: [%i]\n", event);
#endif
      break;
  }
}
#endif