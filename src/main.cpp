#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include "config.h"
#include "smarthome.h"
#include <SmarthaurinHassEmbed.h>

ADC_MODE(ADC_VCC);

BME280 bme280;
SmarthaurinHassEmbed smahe;

#if STATIC_IP == 1
IPAddress local_IP(IP);
IPAddress gateway(GATEWAY);
IPAddress subnet(SUBNET);
IPAddress primaryDNS(PRIMARY_DNS);
#endif
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

char client_id[19];
char base_topic[30];
char topic_state[50];

// RTC is arranged into 4 byte blocks,
// so we have to introduce some padding.
struct {
  uint32_t crc32;   // 4 bytes
  uint8_t channel;  // 1 byte,   5 in total
  uint8_t bssid[6]; // 6 bytes, 11 in total
  uint8_t padding;  // 1 byte,  12 in total
} rtcWifiData;

#if DEBUG == 1
  #define Sprint(a) (Serial.print(a))
  #define Sprintln(a) (Serial.println(a))
  #define Sprintf(a,b) (Serial.printf(a,b))
#else
  #define Sprint(a)
  #define Sprintln(a)
  #define Sprintf(a,b)
#endif

void goodnightEsp(uint32_t sec) {
  delay(1);
  ESP.deepSleep(sec * 1000000ULL, WAKE_RF_DISABLED);
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
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) bit = !bit;

      crc <<= 1;
      if (bit) crc ^= 0x04c11db7;
    }
  }
  return crc;
}

void setup() {
  unsigned long timestamp;
  char topic_config[50];
  char msg_config[500];

#if DEBUG == 1
  Serial.begin(115200);
#endif
  Sprintln("\nStarting measurement iteration");

  // Read WiFi settings from RTC memory
  bool rtcDataValid = false;
  if( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcWifiData, sizeof( rtcWifiData ) ) ) {
    // Calculate and compare the CRC of read data from RTC memory, but skip the first 4 bytes, that's the checksum.
    uint32_t crc = calculateCRC32( ((uint8_t*)&rtcWifiData) + 4, sizeof( rtcWifiData ) - 4 );
    if( crc == rtcWifiData.crc32 ) {
      rtcDataValid = true;
    }
  }

  // reanable wifi radio
  WiFi.forceSleepBegin();
  delay(1);
  WiFi.forceSleepWake();
  delay(1);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
#if STATIC_IP == 1
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, INADDR_NONE)) {
    Sprintln("Wifi failed to configure");
  }
#endif

  if (rtcDataValid) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, rtcWifiData.channel, rtcWifiData.bssid, true);
    Sprint("Starting specific wifi connection: ");
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Sprint("Starting general wifi connection: ");
  }
  
  if (WiFi.waitForConnectResult(WIFI_TIMEOUT_SEC*1000) == WL_CONNECTED) {
    Sprintln("success");
  } else {
    Sprintln("failed, retrying");

    WiFi.disconnect();
    delay(10);
    WiFi.forceSleepBegin();
    delay(10);
    WiFi.forceSleepWake();
    delay(10);

    Sprint("Starting general wifi connection: ");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    if (WiFi.waitForConnectResult(WIFI_TIMEOUT_SEC*1000) != WL_CONNECTED) {
      Sprintln("failed, sleeping");
      goodnightEsp(SLEEP_TIME_ERROR_SEC);
    }
    Sprintln("success");
  }

  // Write current connection info into RTC
  rtcWifiData.channel = WiFi.channel();
  memcpy(rtcWifiData.bssid, WiFi.BSSID(), 6);
  rtcWifiData.crc32 = calculateCRC32( ((uint8_t*)&rtcWifiData) + 4, sizeof(rtcWifiData) - 4);
  ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcWifiData, sizeof(rtcWifiData));
  
  if (snprintf(client_id, sizeof(client_id), "esp8266-%08X", ESP.getChipId()) >= (int) sizeof(client_id)) {
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

  // if (snprintf(topic_state, sizeof(topic_state), TOPIC_HASS_STATE, client_id) >= (int) sizeof(topic_state)) {
  //   Sprintln("Mqtt general state topic cannot be constructed");
  // };

  if (!smahe.getTopicSensorState(topic_state, sizeof(topic_state))){
    Sprintln("Mqtt general state topic cannot be constructed");
  };

  if (snprintf(msg_config, sizeof(msg_config), PAYLOAD_HASS_CONFIG_TEMP, topic_state, client_id, client_id, client_id) >= (int) sizeof(msg_config)) {
    Sprintln("Mqtt temp config msg cannot be constructed");
  };
  if (snprintf(topic_config, sizeof(topic_config), TOPIC_HASS_CONFIG_TEMP, client_id) >= (int) sizeof(topic_config)) {
    Sprintln("Mqtt config topic cannot be constructed");
  };
  // publish temperature configuration
  publishMsg(topic_config, msg_config);

  // if (snprintf(msg_config, sizeof(msg_config), PAYLOAD_HASS_CONFIG_HUM, topic_state, client_id, client_id) >= (int) sizeof(msg_config)) {
  //   Sprintln("Mqtt humidity config msg cannot be constructed");
  // };
  // if (snprintf(topic_config, sizeof(topic_config), TOPIC_HASS_CONFIG_HUM, client_id) >= (int) sizeof(topic_config)) {
  //   Sprintln("Mqtt config topic cannot be constructed");
  // };
  // // publish humidity configuration
  // publishMsg(topic_config, msg_config);

  // if (snprintf(msg_config, sizeof(msg_config), PAYLOAD_HASS_CONFIG_BATT, topic_state, client_id, client_id) >= (int) sizeof(msg_config)) {
  //   Sprintln("Mqtt battery config msg cannot be constructed");
  // };

  // if (snprintf(topic_config, sizeof(topic_config), TOPIC_HASS_CONFIG_BATT, client_id) >= (int) sizeof(topic_config)) {
  //   Sprintln("Mqtt config topic cannot be constructed");
  // };
  // // publish battery configuration
  // publishMsg(topic_config, msg_config);

  // TODO add attributes like uptime, wifi signal, MAC
  // if (snprintf(topic_config, sizeof(topic_config), "homeassistant/sensor/%s/attributes", client_id) >= (int) sizeof(topic_config)) {
  //   Sprintln("Mqtt attribute topic cannot be constructed");
  // };
  // publishMsg(topic_config, PAYLOAD_HASS_ATTR);

  Wire.begin();
  if (Wire.status() == I2C_OK) {
    Wire.setClock(400000);
    Sprintln("I2C runnig at 400KHz");
  } else {
    Sprintln("Cannot start I2C");
  }

  Sprint("Initialising BME280: ");
  bme280.setI2CAddress(0x76);

  if (bme280.beginI2C()) {
    bme280.setMode(MODE_SLEEP);
    bme280.setPressureOverSample(0);  // disable pressure measurements
    bme280.setTempOverSample(1);
    bme280.setHumidityOverSample(1);
    bme280.setFilter(0);
    Sprintln("success");
  } else {
    // TODO FIX THIS
    // char topic[100];
    // if (createTopic(topic, TOPIC_SUFFIX_ERROR, sizeof(topic)) == 0) {
    //   publishMsg(topic, "bme280_failure");
    // };
    Sprintln("failed");
    goodnightEsp(SLEEP_TIME_ERROR_SEC);
  }
}

void loop() {
  int voltage_percentage = 0;
  char msg[100];

  bme280.setMode(MODE_FORCED);

  Sprint("Measuring battery voltage: ");
  float voltage = ESP.getVcc() / 1024.0f;
  Sprint(voltage);

  if (voltage < BATTERY_CR123A_LOW) {
    voltage_percentage = 0;
  } else if (voltage > BATTERY_CR123A_HIGH) {
    voltage_percentage = 100;
  } else {
    voltage_percentage = (voltage - BATTERY_CR123A_LOW) * 100 / (BATTERY_CR123A_HIGH - BATTERY_CR123A_LOW);
  }
  Sprint(" = ");
  Sprintln(voltage_percentage);
  
  Sprintln("Starting temp hum measurement");
  while(bme280.isMeasuring() == true) ; // Wait until measurement is done

  if (snprintf(msg, sizeof(msg), PAYLOAD_HASS_STATE, bme280.readTempC(), bme280.readFloatHumidity(), voltage_percentage) >= (int) sizeof(msg)) {
    Sprintln("Mqtt state payload cannot be constructed");
  };
  publishMsg(topic_state, msg);


  mqttClient.loop();
  // delay(100);  // leave some time for pubsubclient

  mqttClient.disconnect();
  WiFi.disconnect(true);

  Sprintln("All done, sleeping...");
  goodnightEsp(SLEEP_TIME_REGULAR_SEC); 
}