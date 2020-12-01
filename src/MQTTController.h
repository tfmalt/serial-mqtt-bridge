
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

struct MQTTConfig {
  std::string server;
  uint16_t port;
  std::string clientname;
  std::string username;
  std::string password;
  uint16_t buffersize;
};

class MQTTController {
 public:
  const char* version = VERSION;

  MQTTController(){};

  ~MQTTController(){};

  MQTTController& setWiFiController(WiFiController& w);
  MQTTController& setup();
  MQTTController& setup(const char* server,
                        uint16_t port,
                        const char* clientname,
                        const char* user,
                        const char* pass,
                        uint16_t buffer);

  MQTTController& onReady(OnReadyFunction callback);
  MQTTController& onDisconnect(OnDisconnectFunction callback);
  MQTTController& onError(OnErrorFunction callback);
  MQTTController& onMessage(OnMessageFunction callback);

  void loop();
  void connect();

  MQTTController& setLastwillTopic(const char*, uint8_t, bool, const char*);
  MQTTController& enableVerboseOutput();
  MQTTController& enableVerboseOutput(bool v);
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
  MQTTConfig config;
  char* topic_last_will;
  uint8_t last_will_qos;
  bool last_will_retain;
  char* last_will_text;

  std::vector<std::string> subscriptions;

  bool VERBOSE = false;

  PubSubClient client;
  WiFiController* wifiCtrl;

  OnMessageFunction _onMessage;
  OnReadyFunction _onReady;
  OnDisconnectFunction _onDisconnect;
  OnErrorFunction _onError;

  void checkWiFiConnection();
  void checkMQTTConnection();
};
#endif  // MQTTController_h