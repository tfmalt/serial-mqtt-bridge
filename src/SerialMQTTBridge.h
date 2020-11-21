#ifndef SERIALMQTTBRIDGE_H
#define SERIALMQTTBRIDGE_H

#include <Arduino.h>
#include <MQTTController.h>
#include <SerialTransfer.h>
#include <SoftwareSerial.h>
#include <WiFiController.h>
#include <functional>
#include <string>

typedef std::function<void()> OnReadyFunction;
typedef std::function<void(std::string)> OnDisconnectFunction;
typedef std::function<void(std::string)> OnErrorFunction;
typedef std::function<void(std::string, std::string)> OnMessageFunction;

enum MQTTMessageType {
  CONNECT,
  CONNECT_ACK,
  SUBSCRIBE,
  UNSUBSCRIBE,
  STATUS_NO_SUB,
  STATUS_OK,
  PUBLISH,
  MESSAGE
};

enum MQTTQoS { AT_MOST_ONCE = 0, AT_LEAST_ONCE = 1, EXACTLY_ONCE = 2 };

struct MQTTMessage {
  uint8_t type;
  uint8_t count;
  uint8_t qos;
  char topic[32];
  char message[256];
};

struct SMBMQTTConfig {
  std::string server;
  uint16_t port;
  std::string clientname;
  std::string username;
  std::string password;
};

struct SMBWiFiConfig {
  std::string ssid;
  std::string psk;
  std::string hostname;
};

struct SMBSerialConfig {
  uint8_t rxPin;
  uint8_t txPin;
  uint32_t baudrate;
};

struct SMBConfig {
  SMBMQTTConfig mqtt;
  SMBWiFiConfig wifi;
  SMBSerialConfig serial;
};

class SerialMQTTBridge {
 private:
  SoftwareSerial swSer;
  SerialTransfer rxtx;
  WiFiController wifi;
  MQTTController mqtt;
  SMBConfig _config;
  bool _verbose;

  // ==========================================================================
  // MQTT Event Handlers
  // ==========================================================================
  void handleReady() {
    if (_verbose) {
      Serial.println("[bridge] ||| mqtt connection ready.");
    }

    if (mqtt.subscriptionsCount() == 0) {
      MQTTMessage msg = {.type = MQTTMessageType::STATUS_NO_SUB, .count = 0};
      strcpy(msg.topic, "NO SUBSCRIPTIONS");
      strcpy(msg.message, "");

      rxtx.sendDatum(msg);
    }
  }

  void handleError(std::string msg) {
    if (_verbose) {
      Serial.printf("[bridge] !!! mqtt error: %s\n", msg.c_str());
    }
  }

  void handleMessage(std::string topic, std::string message) {
    if (_verbose) {
      Serial.printf("[bridge] <<< topic: %s, message: %s ", topic.c_str(),
                    message.c_str());
    }

    MQTTMessage msg;
    msg.type = MQTTMessageType::MESSAGE;
    msg.count = mqtt.subscriptionsCount();
    strcpy(msg.topic, topic.c_str());
    strcpy(msg.message, message.c_str());

    rxtx.sendDatum(msg);

    if (_verbose) {
      Serial.println("... done");
    }
  }

  void handleDisconnect(std::string msg) {
    if (_verbose) {
      Serial.printf("[bridge] !!! mqtt disconnect: %s\n", msg.c_str());
    }
  }

  // ==========================================================================
  // Serial Event Handlers
  // ==========================================================================
  void handleConnect() {
    MQTTMessage msg;
    msg.type = MQTTMessageType::CONNECT_ACK;
    msg.count = mqtt.subscriptionsCount();
    strcpy(msg.topic, "CONNECT_ACK");
    strcpy(msg.message, "");

    if (_verbose) {
      Serial.printf("[bridge] <<< CONNECT ACK: topics count: %i\n", msg.count);
    }
    rxtx.sendDatum(msg);
  }

  uint32_t lastHeartbeat = 0;
  void handleHeartbeat() {
    // Send status ok every 10 seconds
    if (millis() - lastHeartbeat >= 10000) {
      MQTTMessage msg;

      msg.count = mqtt.subscriptionsCount();
      msg.type = (msg.count > 0) ? MQTTMessageType::STATUS_OK
                                 : MQTTMessageType::STATUS_NO_SUB;
      strcpy(msg.topic, msg.count > 0 ? "OK" : "NO SUBSCRIPTIONS");
      rxtx.sendDatum(msg);
      lastHeartbeat = millis();
      if (_verbose) {
        Serial.printf("[bridge] <<< heartbeat. topics count: %i\n", msg.count);
      }
    }
  }

 public:
  SerialMQTTBridge(){};

  SerialMQTTBridge& setup(SMBConfig config) {
    _config.wifi.hostname = config.wifi.hostname;
    _config.wifi.ssid = config.wifi.ssid;
    _config.wifi.psk = config.wifi.psk;
    _config.mqtt.server = config.mqtt.server;
    _config.mqtt.port = config.mqtt.port;
    _config.mqtt.clientname = config.mqtt.clientname;
    _config.mqtt.username = config.mqtt.username;
    _config.mqtt.password = config.mqtt.password;
    _config.serial.baudrate = config.serial.baudrate;
    _config.serial.rxPin = config.serial.rxPin;
    _config.serial.txPin = config.serial.txPin;

    if (_verbose) {
      Serial.printf("[bridge] ||| running setup: sw serial rx: %i, tx: %i\n",
                    _config.serial.rxPin, _config.serial.txPin);
      mqtt.enableVerboseOutput();
    }

    Serial.println("getting ready to setup software serial");
    swSer.begin(_config.serial.baudrate, SWSERIAL_8N1, _config.serial.rxPin,
                _config.serial.txPin);

    rxtx.begin(swSer);

    mqtt.setup();
    // mqtt.setLastwillTopic(topic_status.c_str(), 0, true, "Disconnected");

    mqtt.onReady([this]() { this->handleReady(); });
    mqtt.onDisconnect([this](std::string msg) { this->handleDisconnect(msg); });
    mqtt.onError([this](std::string err) { this->handleError(err); });
    mqtt.onMessage([this](std::string topic, std::string message) {
      this->handleMessage(topic, message);
    });

    return *this;
  };

  SerialMQTTBridge& setLastWillTopic(const char* topic,
                                     uint8_t qos,
                                     bool retain,
                                     const char* msg) {
    mqtt.setLastwillTopic(topic, qos, retain, msg);

    return *this;
  }

  void loop() {
    mqtt.loop();

    if (rxtx.available()) {
      MQTTMessage msg;
      rxtx.rxObj(msg);

      switch (msg.type) {
        case MQTTMessageType::MESSAGE: {
          if (_verbose) {
            Serial.printf("[bridge] >>> topic: %s, message: %s\n", msg.topic,
                          msg.message);
            mqtt.publish(msg.topic, msg.message);

            break;
          }
        }

        case MQTTMessageType::CONNECT: {
          Serial.printf("[bridge] >>> %s\n", msg.topic);
          handleConnect();
          break;
        }

        case MQTTMessageType::SUBSCRIBE: {
          Serial.printf("[bridge] >>> SUBSCRIBE: %s\n", msg.topic);
          mqtt.subscribe(msg.topic, true);
          break;
        }
        default: {
          Serial.printf("[bridge] ??? unknown type: %i, topic: %s\n", msg.type,
                        msg.topic);
        }
      }
    }

    handleHeartbeat();
  }

  SerialMQTTBridge& enableVerboseOutput(bool v = true) {
    _verbose = v;
    return *this;
  }
};

// std::string mqtt_client(MQTT_USER);
// std::string topic_information = "/" + mqtt_client + MQTT_TOPIC_INFORMATION;
// std::string topic_status = "/" + mqtt_client + MQTT_TOPIC_STATUS;
// std::string topic_command = "/" + mqtt_client + MQTT_TOPIC_COMMAND;
// std::string topic_query = "/" + mqtt_client + MQTT_TOPIC_QUERY;

#endif  // SERIALMQTTBRIDGE_H