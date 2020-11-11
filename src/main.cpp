#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include "config.h"
#include "smarthome.h"

#define BATTERY_ADC_PIN 34

BME280 bme280;
bool bme_running = false;

#if STATIC_IP == 1
IPAddress local_IP(IP);
IPAddress gateway(GATEWAY);
IPAddress subnet(SUBNET);
IPAddress primaryDNS(PRIMARY_DNS);
#endif
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
char client_id[15];
char base_topic[30];

unsigned long timestamp;

#if DEBUG == 1
  #define Sprint(a) (Serial.print(a))
  #define Sprintln(a) (Serial.println(a))
  #define Sprintf(a,b) (Serial.printf(a,b))
#else
  #define Sprint(a)
  #define Sprintln(a)
  #define Sprintf(a,b)
#endif

uint32_t espGetChipId() {
  uint32_t chip_id = 0;
  for (uint8_t i = 0; i < 17; i = i + 8) {
      chip_id |= ((ESP.getEfuseMac() >> (40u - i)) & 0xffu) << i;
  }
  return chip_id;
}

void goodnightEsp(uint32_t sec) {
  esp_sleep_enable_timer_wakeup(sec * 1000000ULL);
  esp_deep_sleep_start();
}

int createTopic(char *dest, const char *suffix, size_t dest_length) {
  strlcpy(dest, base_topic, dest_length);
  strlcat(dest, suffix, dest_length);
  return 0;
}

void publishMsg(const char *topic, const char *msg) {
  Sprintf("Publishing to topic: %s, message: ", topic);
  Sprintln(msg);
  if (!mqttClient.publish(topic, msg)) {
      Sprintln("Publishing failed");
  };
  // delay(50);
}

void setup() {
#if DEBUG == 1
  Serial.begin(115200);
#endif
  Sprintln("\n Starting measurement iteration");

#if STATIC_IP == 0
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, INADDR_NONE)) {
    Sprintln("Wifi failed to configure");
  }
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Sprint("Starting wifi connection");
  timestamp = millis();
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Sprint(".");
      if (millis() > (timestamp + WIFI_TIMEOUT_SEC*1000)) {
          Sprintln("");
          Sprintln("Wifi connect timeout, sleeping...");
          goodnightEsp(SLEEP_TIME_ERROR_SEC);
      }
  }
  Sprintln("");
  Sprintln("Wifi connection was successfully created");

  
  if (snprintf(client_id, sizeof(client_id), "ESP-%08X", espGetChipId()) >= (int) sizeof(client_id)) {
    Sprintln("Mqtt client id cannot be constructed");
  };
  
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  if (!mqttClient.setBufferSize(MQTT_BUFFER_SIZE)) {
    Sprintln("Mqtt buffer cannot be resized");
  }

  Sprint("Starting mqtt connection: ");
  timestamp = millis();
  while (!mqttClient.connected()) {
    if (mqttClient.connect(client_id, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println("");
    }
    if (millis() > (timestamp + MQTT_TIMEOUT_SEC*1000)) {
          Sprintln("");
          Sprintln("Mqtt connect timeout, sleeping...");
          goodnightEsp(SLEEP_TIME_ERROR_SEC);
      }
  }

  if (snprintf(base_topic, sizeof(base_topic), TOPIC_BASE, client_id) >= (int) sizeof(base_topic)) {
    Sprintln("Mqtt base topic cannot be constructed");
  };


  if (Wire.begin()) {
    Wire.setClock(400000);
    Sprintln("I2C runnig at 400KHz");
  } else {
    Sprintln("Cannot start I2C");
  }

  Sprint("Initialising BME280: ");
  bme280.setI2CAddress(0x76);
  bme_running = bme280.beginI2C();

  if (bme_running) {
    bme280.setMode(MODE_SLEEP);
    bme280.setPressureOverSample(0);  // disable pressure measurements
    bme280.setTempOverSample(1);
    bme280.setHumidityOverSample(1);
    bme280.setFilter(0);
    Sprintln("success");
  } else {
    char topic[100];
    if (createTopic(topic, TOPIC_SUFFIX_ERROR, sizeof(topic)) == 0) {
      publishMsg(topic, "bme280_failure");
    };
    Sprintln("failed");
    // goodnightEsp(SLEEP_TIME_ERROR_SEC);
  }  
}

void loop() {
  char topic[100];

  bme280.setMode(MODE_FORCED);

  Sprint("Measuring battery voltage: ");
  float voltage = analogRead(BATTERY_ADC_PIN) * 3.3 / 4095;
  Sprintln(voltage);
  if (createTopic(topic, TOPIC_SUFFIX_BATTERY, sizeof(topic)) == 0) {
    char batt[8];
    snprintf(batt, sizeof(batt), "%.2f", voltage);
    publishMsg(topic, batt);
  };

  Sprintln("Starting temp hum measurement");
  while(bme280.isMeasuring() == true) ; // Wait until measurement done

  Sprintln("Publishing temperature");
  if (createTopic(topic, TOPIC_SUFFIX_TEMPERATURE, sizeof(topic)) == 0) {
    char temp[8];
    snprintf(temp, sizeof(temp), "%.1f", bme280.readTempC());
    publishMsg(topic, temp);
  };

  Sprintln("Publishing humidity");
  if (createTopic(topic, TOPIC_SUFFIX_HUMIDITY, sizeof(topic)) == 0) {
    char hum[8];
    snprintf(hum, sizeof(hum), "%.0f", bme280.readFloatHumidity());
    publishMsg(topic, hum);
  };

  delay(100);  // leave some time for pubsubclient

  mqttClient.disconnect();
  WiFi.disconnect();

  Sprintln("All done, sleeping...");
  goodnightEsp(SLEEP_TIME_REGULAR_SEC); 
}