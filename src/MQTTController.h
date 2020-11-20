
#ifndef MQTTController_h
#define MQTTController_h

#include <Arduino.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <WiFiController.h>
#include <WiFiUdp.h>
#include <algorithm>
#include <functional>
#include <string>
#include <vector>

typedef std::function<void()> OnReadyFunction;
typedef std::function<void(std::string)> OnDisconnectFunction;
typedef std::function<void(std::string)> OnErrorFunction;
typedef std::function<void(std::string, std::string)> OnMessageFunction;

class MQTTController {
 public:
  const char* version = VERSION;

  MQTTController(const char* wifi_ssid,
                 const char* wifi_psk,
                 const char* mqtt_server,
                 const char* mqtt_user,
                 const char* mqtt_pass,
                 const uint16_t mqtt_port = 1883,
                 const char* wifi_hostname = "wifimqtt",
                 const char* mqtt_client = "wifimqtt")
      : wifi_ssid(wifi_ssid),
        wifi_psk(wifi_psk),
        wifi_hostname(wifi_hostname),
        mqtt_server(mqtt_server),
        mqtt_user(mqtt_user),
        mqtt_pass(mqtt_pass),
        mqtt_client(mqtt_client),
        mqtt_port(mqtt_port){};

  ~MQTTController(){};

  MQTTController& setup();
  MQTTController& onReady(OnReadyFunction callback);
  MQTTController& onDisconnect(OnDisconnectFunction callback);
  MQTTController& onError(OnErrorFunction callback);
  MQTTController& onMessage(OnMessageFunction callback);

  void loop();
  void connect();

  MQTTController& setLastwillTopic(const char*, uint8_t, bool, const char*);
  MQTTController& enableVerboseOutput();
  MQTTController& subscribe(std::string topic);
  MQTTController& subscribe(const char* topic);
  MQTTController& subscribe(std::string topic, bool immediately);
  MQTTController& subscribe(const char* topic, bool immediately);
  MQTTController& reSubscribe();
  uint8_t subscriptionsCount();  // { return subscriptions.size(); }

  bool publish(const char* topic, const char* message);
  bool publish(std::string topic, std::string message);
  // void publishInformation(const char *message);
  void publishInformationData();

 private:
  const char* wifi_ssid;
  const char* wifi_psk;
  const char* wifi_hostname;
  const char* mqtt_server;
  const char* mqtt_user;
  const char* mqtt_pass;
  const char* mqtt_client;
  const uint16_t mqtt_port;

  char* topic_last_will;
  uint8_t last_will_qos;
  bool last_will_retain;
  char* last_will_text;

  std::vector<std::string> subscriptions;

  bool VERBOSE = false;

  PubSubClient client;
  WiFiController wifiCtrl;

  OnMessageFunction _onMessage;
  OnReadyFunction _onReady;
  OnDisconnectFunction _onDisconnect;
  OnErrorFunction _onError;

  void checkWiFiConnection();
  void checkMQTTConnection();
};
#endif  // MQTTController_h