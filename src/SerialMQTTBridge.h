#ifndef SERIALMQTTBRIDGE_H
#define SERIALMQTTBRIDGE_H

#include <Arduino.h>
#include <MQTTController.h>
#include <SerialTransfer.h>
#include <SoftwareSerial.h>

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

SoftwareSerial swSer;
SerialTransfer rxtx;

// std::string mqtt_client(MQTT_USER);
// std::string topic_information = "/" + mqtt_client + MQTT_TOPIC_INFORMATION;
// std::string topic_status = "/" + mqtt_client + MQTT_TOPIC_STATUS;
// std::string topic_command = "/" + mqtt_client + MQTT_TOPIC_COMMAND;
// std::string topic_query = "/" + mqtt_client + MQTT_TOPIC_QUERY;

#endif  // SERIALMQTTBRIDGE_H