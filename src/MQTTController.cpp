
#include "MQTTController.hpp"

#include <NTPClient.h>
#include <WiFiUdp.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3598, 60000);

/**
 * Setup this instance of controller
 */
MQTTController& MQTTController::setup() {
  if (VERBOSE) {
    Serial.printf("[mqtt] ||| Setting up client: %s:%i\n", mqtt_server,
                  mqtt_port);
    wifiCtrl.enableVerboseOutput();
  }

  wifiCtrl.connect(wifi_ssid, wifi_psk, wifi_hostname);
  timeClient.begin();
  timeClient.update();

  WiFiClient& wifi = wifiCtrl.getWiFiClient();
  bool setbuf = client.setBufferSize(MQTT_BUFFER_SIZE);

  if (VERBOSE) {
    Serial.printf("[mqtt] ||| NTP time client started: %s.\n",
                  timeClient.getFormattedTime().c_str());
    Serial.printf("[mqtt] ||| Set MQTT buffer to %i: %s\n", MQTT_BUFFER_SIZE,
                  (setbuf) ? "true" : "false");
  }

  client.setClient(wifi);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(
      [this](char* p_topic, byte* p_message, unsigned int p_length) {
        std::string topic = p_topic;
        std::string message(reinterpret_cast<const char*>(p_message), p_length);

        if (this->_onMessage != nullptr) {
          this->_onMessage(topic, message);
        }
      });

  return *this;
}

/**
 * Connect
 */
void MQTTController::connect() {
  while (!client.connected()) {
    if (VERBOSE) {
      Serial.printf("[mqtt] ||| Attempting connection to %s:%i as \"%s\" ...",
                    mqtt_server, mqtt_port, mqtt_user);
    }

    if (!client.connect(mqtt_client, mqtt_user, mqtt_pass, topic_last_will,
                        last_will_qos, last_will_retain, last_will_text)) {
      if (VERBOSE) {
        Serial.print(" failed: ");
        Serial.println(client.state());
      }
      delay(5000);
    }
  }

  if (VERBOSE) {
    Serial.println(" connected.");
  }

  timeClient.update();

  this->reSubscribe();

  if (this->_onReady != nullptr) {
    this->_onReady();
  }
}

MQTTController& MQTTController::reSubscribe() {
  for (auto topic : subscriptions) {
    bool res = client.subscribe(topic.c_str());
    if (VERBOSE) {
      Serial.printf("[mqtt] >>> subscribed to: %s: %s\n", topic.c_str(),
                    res ? "true" : "false");
    }
  }
}

/**
 * @brief the main loop
 *
 */
void MQTTController::loop() {
  checkWiFiConnection();
  checkMQTTConnection();

  client.loop();
}

/**
 * @brief Verifies the MQTT connection is up and running, if not it reconnects.
 *
 */
void MQTTController::checkMQTTConnection() {
  if (!client.connected()) {
    std::string msg = "MQTT broker not connected: " + std::string{mqtt_server};
    if (VERBOSE) {
      Serial.printf("[mqtt] ||| %s\n", msg.c_str());
    }
    if (this->_onDisconnect != nullptr) {
      this->_onDisconnect(msg);
    }
    connect();
  }
}

/**
 * @brief verifies the wifi connection is up and running. if not does a restart.
 * Could probably be implementeed more gracefully.
 */
void MQTTController::checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (VERBOSE) {
      Serial.println("[mqtt] ||| Restarting because WiFi not connected.");
    }
    if (this->_onError != nullptr) {
      this->_onError("Restarting because WiFi not connected.");
    }
    delay(1000);
    ESP.restart();
    return;
  }
}

// ==========================================================================
// Event handler callbacks
// ==========================================================================
MQTTController& MQTTController::onMessage(
    std::function<void(std::string, std::string)> callback) {
  this->_onMessage = callback;
  return *this;
}

MQTTController& MQTTController::onReady(std::function<void()> callback) {
  this->_onReady = callback;
  return *this;
}

MQTTController& MQTTController::onDisconnect(
    std::function<void(std::string msg)> callback) {
  this->_onDisconnect = callback;
  return *this;
}

MQTTController& MQTTController::onError(
    std::function<void(std::string error)> callback) {
  this->_onError = callback;
  return *this;
}
// ==========================================================================
// End of Event handler callbacks
// ==========================================================================

MQTTController& MQTTController::enableVerboseOutput() {
  VERBOSE = true;
  return *this;
}

MQTTController& MQTTController::setLastwillTopic(const char* topic,
                                                 uint8_t qos = 0,
                                                 bool retain = false,
                                                 const char* message = "") {
  topic_last_will = (char*)topic;
  last_will_qos = qos;
  last_will_retain = retain;
  last_will_text = (char*)message;

  return *this;
}

bool MQTTController::publish(const char* topic, const char* message) {
  // TODO: Add assertion that topic is corret.
  return client.publish(topic, message, false);
}

bool MQTTController::publish(std::string topic, std::string message) {
  return publish(topic.c_str(), message.c_str());
}

/**
 * publish information data
 * compile some useful metainformation about the system
 */
void MQTTController::publishInformationData() {
  char* msg;

  WiFiClient& wifi = wifiCtrl.getWiFiClient();

  asprintf(&msg,
           "{\"time\": \"%s\", \"version\": \"%s\", \"ip\": \"%s\", "
           "\"uptime\": %lu, \"memory\": %d }",
           timeClient.getFormattedTime().c_str(), version,
           wifi.localIP().toString().c_str(), ESP.getFreeHeap());

  const char* message = msg;

  std::string topic =
      "/" + std::string{MQTT_CLIENT} + std::string{MQTT_TOPIC_INFORMATION};
  client.publish(topic.c_str(), message, false);

  free(msg);
}

MQTTController& MQTTController::subscribe(std::string topic) {
  subscriptions.push_back(topic);

  std::vector<std::string>::iterator it;

  it = std::unique(subscriptions.begin(), subscriptions.end());
  subscriptions.resize(std::distance(subscriptions.begin(), it));

  return *this;
}

MQTTController& MQTTController::subscribe(std::string topic, bool immediately) {
  this->subscribe(topic);

  if (immediately == true) {
    if (client.connected()) {
      bool res = client.subscribe(topic.c_str());
      if (VERBOSE) {
        Serial.printf("[mqtt] ||| subscribed to: %s: %s\n", topic.c_str(),
                      res ? "true" : "false");
      }
    }
  }

  return *this;
}

MQTTController& MQTTController::subscribe(const char* topic) {
  return subscribe(std::string{topic});
}

MQTTController& MQTTController::subscribe(const char* topic, bool immediately) {
  return subscribe(std::string{topic}, immediately);
}

uint8_t MQTTController::subscriptionsCount() {
  return (uint8_t)subscriptions.size();
}