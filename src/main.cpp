#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <EspChipId.h>
#include <MyBattery.h>
#include <PubSubClient.h>
#include <SerPrint.h>
#include <SparkFunBME280.h>
#include <Wire.h>
#include "config.h"
#include "smarthome.h"

ADC_MODE(ADC_VCC);

BME280 bme280;
#if STATIC_IP == 1
IPAddress local_IP(IP);
IPAddress gateway(GATEWAY);
IPAddress subnet(SUBNET);
IPAddress primaryDNS(PRIMARY_DNS);
#endif
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

char topic_state[70];

// RTC is arranged into 4 byte blocks,
// so we have to introduce some padding.
struct {
  uint32_t crc32;   // 4 bytes
  uint8_t channel;  // 1 byte,   5 in total
  uint8_t bssid[6]; // 6 bytes, 11 in total
  uint8_t padding;  // 1 byte,  12 in total
} rtcWifiData;

void goodnightEsp(uint32_t sec) {
  delay(2000);
  ESP.deepSleep(sec * 1000000ULL, WAKE_RF_DISABLED);
}

void publishMsg(const char *t, const char *m, bool r = false) {
  SerPrintf("Publishing to topic: %s, message: ", t);
  SerPrintln(m);
  if (!mqttClient.publish(t, m, r)) {
      SerPrintln("ERROR, Publishing failed");
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
  char client_id[19], topic_base[50];

  SerBegin(115200);
  SerPrintln("\nStarting measurement iteration");

  // Read WiFi settings from RTC memory
  bool rtcDataValid = false;
  if(ESP.rtcUserMemoryRead(0, (uint32_t*)&rtcWifiData, sizeof(rtcWifiData))) {
    // Calculate and compare the CRC of read data from RTC memory, but skip the first 4 bytes, that's the checksum.
    uint32_t crc = calculateCRC32(((uint8_t*)&rtcWifiData) + 4, sizeof(rtcWifiData) - 4);
    if(crc == rtcWifiData.crc32) {
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
    SerPrintln("ERROR, failed to configure Wifi");
  }
#endif

  if (rtcDataValid) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, rtcWifiData.channel, rtcWifiData.bssid, true);
    SerPrint("Starting specific wifi connection: ");
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    SerPrint("Starting general wifi connection: ");
  }
  
  if (WiFi.waitForConnectResult(WIFI_TIMEOUT_SEC*1000) == WL_CONNECTED) {
    SerPrintln("success");
  } else {
    SerPrintln("failed, retrying");

    WiFi.disconnect();
    delay(10);
    WiFi.forceSleepBegin();
    delay(10);
    WiFi.forceSleepWake();
    delay(10);

    SerPrint("Starting general wifi connection: ");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    if (WiFi.waitForConnectResult(WIFI_TIMEOUT_SEC*1000) != WL_CONNECTED) {
      SerPrintln("failed, sleeping");
      goodnightEsp(SLEEP_TIME_ERROR_SEC);
    }
    SerPrintln("success");
  }

  // Write current connection info into RTC
  rtcWifiData.channel = WiFi.channel();
  memcpy(rtcWifiData.bssid, WiFi.BSSID(), 6);
  rtcWifiData.crc32 = calculateCRC32(((uint8_t*)&rtcWifiData) + 4, sizeof(rtcWifiData) - 4);
  ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcWifiData, sizeof(rtcWifiData));
  
  if (snprintf(client_id, sizeof(client_id), DEVICE_ID, EspChipId.get()) >= (int) sizeof(client_id)) {
    SerPrintln("Mqtt client id cannot be constructed");
  };
  
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  if (!mqttClient.setBufferSize(MQTT_BUFFER_SIZE)) {
    SerPrintln("Mqtt buffer cannot be resized");
  }

  SerPrint("Starting mqtt connection: ");
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
          SerPrintln("");
          SerPrintln("Mqtt connect timeout, sleeping...");
          goodnightEsp(SLEEP_TIME_ERROR_SEC);
      }
  }

  // CONFIGURE HASS STATE TOPICS
  if (snprintf(topic_base, sizeof(topic_base), HASS_BASE_TOPIC, client_id) >= (int) sizeof(topic_base)) {
    SerPrintln("ERROR, base topic cannot be constructed");
  }
  if (strlcpy(topic_state, topic_base, sizeof(topic_state)) >= sizeof(topic_state)) {
        SerPrintln("ERROR, state topic cannot be copyied");
  }
  if (strlcat(topic_state, "/state", sizeof(topic_state)) >= sizeof(topic_state)) {
    SerPrintln("ERROR, state topic cannot be constructed");
  }

  if (false) {
    char dev_conf[150], mac_addr[18];
    uint8_t mac[6];
    const char *configs[3] = {HASS_CONF_TEMP, HASS_CONF_HUM, HASS_CONF_BATT};
    const char *suffixes[3] = {UID_TEMP_SUFFIX, UID_HUM_SUFFIX, UID_BATT_SUFFIX};

    WiFi.macAddress(mac);
    if (sprintf(mac_addr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) >= (int) sizeof(mac_addr)) {
      SerPrintln("ERROR, mac address cannot be constructed");
    }
    if (snprintf(dev_conf, sizeof(dev_conf), HASS_CONF_DEVICE, mac_addr, EspChipId.get()) >= (int) sizeof(dev_conf)) {
      SerPrintln("ERROR, device config cannot be constructed");
    }

    for(int i=0; i < 3 ; i++) {
      char config_doc[400], topic_config[50];

      if (strlcpy(topic_config, topic_base, sizeof(topic_config)) >= sizeof(topic_config)) {
        SerPrintln("ERROR, base topic cannot be copyied");
      }
      if (strlcat(topic_config, suffixes[i], sizeof(topic_config)) >= sizeof(topic_config)) {
        SerPrintln("ERROR, config topic suffix cannot be constructed");
      }
      if (strlcat(topic_config, "/config", sizeof(topic_config)) >= sizeof(topic_config)) {
        SerPrintln("ERROR, config topic cannot be constructed");
      }

      StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, configs[i]);
        if (error) {
            SerPrint("ERROR, deserialization failed: ");
            SerPrintln(error.f_str());
            return;
        }
        doc["~"] = topic_base;
        doc["unique_id"] = client_id;
        doc["dev"] = serialized(dev_conf);

        if (measureJson(doc) >= sizeof(config_doc)) {
          SerPrintln("ERROR, config document char array too short");
          break;
        }
        serializeJson(doc, config_doc);
        publishMsg(topic_config, config_doc, true);    
    }
  }

  // PUBLISH HASS ATTRIBUTES
  {
    char topic[50], msg[100];
    if (strlcpy(topic, topic_base, sizeof(topic)) >= sizeof(topic)) {
      SerPrintln("ERROR, base topic cannot be copyied");
    }
    if (strlcat(topic, "/attributes", sizeof(topic)) >= sizeof(topic)) {
          SerPrintln("ERROR, attributes topic cannot be constructed");
    }
    if (snprintf(msg, sizeof(msg), HASS_ATTRIBUTE_COLLECTION, WiFi.RSSI()) >= (int) sizeof(msg)) {
      SerPrintln("ERROR, attribute collection cannot be constructed");
    }
    publishMsg(topic, msg);
  }

  Wire.begin();
  if (Wire.status() == I2C_OK) {
    Wire.setClock(400000);
    SerPrintln("I2C runnig at 400KHz");
  } else {
    SerPrintln("Cannot start I2C");
  }

  SerPrint("Initialising BME280: ");
  bme280.setI2CAddress(0x76);

  if (bme280.beginI2C()) {
    bme280.setMode(MODE_SLEEP);
    bme280.setPressureOverSample(0);  // disable pressure measurements
    bme280.setTempOverSample(1);
    bme280.setHumidityOverSample(1);
    bme280.setFilter(0);
    SerPrintln("success");
  } else {
    // TODO FIX THIS
    // char topic[100];
    // if (createTopic(topic, TOPIC_SUFFIX_ERROR, sizeof(topic)) == 0) {
    //   publishMsg(topic, "bme280_failure");
    // };
    SerPrintln("failed");
    goodnightEsp(SLEEP_TIME_ERROR_SEC);
  }
}

void loop() {
  int voltage_percentage = 0;
  char msg[100];
  float voltage;

  bme280.setMode(MODE_FORCED);

  SerPrint("Measuring battery voltage: ");
  voltage = ESP.getVcc() / 1024.0f;
  SerPrint(voltage);
  SerPrint(" = ");
  voltage_percentage = MyBattery.calcCr123Percentage(voltage);
  SerPrintln(voltage_percentage);
  
  SerPrintln("Wating for temp hum measurement");
  while(bme280.isMeasuring() == true) ; // Wait until measurement is done

  if (snprintf(msg, sizeof(msg), HASS_PAYLOAD_STATE, bme280.readTempC(), bme280.readFloatHumidity(), voltage_percentage) >= (int) sizeof(msg)) {
    SerPrintln("Mqtt state payload cannot be constructed");
  };
  publishMsg(topic_state, msg);


  mqttClient.loop();
  // delay(100);  // leave some time for pubsubclient

  mqttClient.disconnect();
  WiFi.disconnect(true);
SerPrintln(millis());
  SerPrintln("All done, sleeping...");
  goodnightEsp(SLEEP_TIME_REGULAR_SEC); 
}