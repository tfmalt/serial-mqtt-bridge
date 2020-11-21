/**
 * @file bridge.cpp
 * @author Thomas Malt (thomas@malt.no)
 * @brief An ESP8266/ESP32 firmware to translate between serial<->mqtt<->wifi to
 * deliver mqtt to arduinos and other devices like teensys that lack them
 * @version 0.1
 * @date 2020-11-18
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <Arduino.h>
#include <MQTTController.h>
#include <SerialMQTTBridge.h>
#include <WiFiController.h>

SerialMQTTBridge bridge;

void setup() {
  SMBConfig config;
  config.hostname = WIFI_HOSTNAME;
  config.mqttclient = MQTT_CLIENT;
  config.mqttserver = MQTT_SERVER;
  config.mqttport = MQTT_PORT;
  config.mqttuser = MQTT_USER;
  config.mqttpass = MQTT_PASS;
  config.ssid = WIFI_SSID;
  config.psk = WIFI_PSK;

  bridge.setup(config);
}

void loop() {}

// ==========================================================================
// Handlers
// ==========================================================================
void handleReady() {
#ifdef DEBUG
  Serial.println("[main] ||| mqtt connection ready.");
#endif
  if (mqtt.subscriptionsCount() == 0) {
    MQTTMessage msg = {.type = MQTTMessageType::STATUS_NO_SUB, .count = 0};
    strcpy(msg.topic, "NO SUBSCRIPTIONS");
    strcpy(msg.message, "");
    rxtx.sendDatum(msg);
  }
}

void handleError(std::string msg) {
#ifdef DEBUG
  Serial.printf("[main] mqtt error: %s\n", msg.c_str());
#endif
}

void handleMessage(std::string topic, std::string message) {
#ifdef DEBUG
  Serial.printf("[main] <<< topic: %s, message: %s ", topic.c_str(),
                message.c_str());
#endif
  MQTTMessage msg;
  msg.type = MQTTMessageType::MESSAGE;
  msg.count = mqtt.subscriptionsCount();
  strcpy(msg.topic, topic.c_str());
  strcpy(msg.message, message.c_str());

  rxtx.sendDatum(msg);

  Serial.println("... done");
}

void handleDisconnect(std::string message) {}

void handleConnect() {
  MQTTMessage msg;
  msg.type = MQTTMessageType::CONNECT_ACK;
  msg.count = mqtt.subscriptionsCount();
  strcpy(msg.topic, "CONNECT_ACK");
  strcpy(msg.message, "");

#ifdef DEBUG
  Serial.printf("[main] <<< CONNECT ACK: topics count: %i\n", msg.count);
#endif
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
#ifdef DEBUG
    Serial.printf("[main] <<< heartbeat. topics count: %i\n", msg.count);
#endif
  }
}

// ==========================================================================
// Setup
// ==========================================================================
void setup() {
  Serial.begin(115200);
  swSer.begin(SWSERIAL_BAUD_RATE, SWSERIAL_8N1, D5, D6);

#ifdef DEBUG
  Serial.println();
  Serial.printf("[main] ||| running setup: sw serial rx: %i, tx: %i\n", D5, D6);
#endif

  rxtx.begin(swSer);

#ifdef DEBUG
  mqtt.enableVerboseOutput();
#endif

  mqtt.setup();
  mqtt.setLastwillTopic(topic_status.c_str(), 0, true, "Disconnected");
  mqtt.onReady(handleReady);
  mqtt.onDisconnect(handleDisconnect);
  mqtt.onError(handleError);
  mqtt.onMessage(handleMessage);
}

// ==========================================================================
// Loop
// ==========================================================================
void loop() {
  mqtt.loop();
  if (rxtx.available()) {
    MQTTMessage msg;
    rxtx.rxObj(msg);

    switch (msg.type) {
      case MQTTMessageType::MESSAGE: {
#ifdef DEBUG
        Serial.printf("[main] >>> topic: %s, message: %s\n", msg.topic,
                      msg.message);
#endif
        mqtt.publish(msg.topic, msg.message);

        break;
      }

      case MQTTMessageType::CONNECT: {
        Serial.printf("[main] >>> %s\n", msg.topic);
        handleConnect();
        break;
      }

      case MQTTMessageType::SUBSCRIBE: {
        Serial.printf("[main] >>> SUBSCRIBE: %s\n", msg.topic);
        mqtt.subscribe(msg.topic, true);
        break;
      }
      default: {
        Serial.printf("[main] ??? unknown type: %i, topic: %s\n", msg.type,
                      msg.topic);
      }
    }
  }

  handleHeartbeat();
}