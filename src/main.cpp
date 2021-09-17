#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <EspChipId.h>
#include <MyBattery.h>
#include <EspNtpTime.h>
#include <PubSubClient.h>
#include <SerPrint.h>
#include <SparkFunBME280.h>
#include <Wire.h>
#include "config.h"
#include "smarthome.h"

#ifdef EPAPER
#include <Adafruit_GFX.h>
#include <Adafruit_BusIO_Register.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Fonts/FreeSansOblique9pt7b.h>
#include <GxEPD2_BW.h>
#include "graphics.h"
#endif

ADC_MODE(ADC_VCC);

BME280 bme280;
BME280_SensorMeasurements measurements;
#if STATIC_IP == 1
IPAddress local_IP(IP);
IPAddress gateway(GATEWAY);
IPAddress subnet(SUBNET);
IPAddress primaryDNS(PRIMARY_DNS);
#endif
EspNtpTime ntptime;
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
#ifdef EPAPER
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS*/ 3, /*DC*/ D3, /*RST*/ D8, /*BUSY*/ D4));
#endif

void goodnightEsp(uint32_t sec) {
  delay(100);
  ESP.deepSleep(sec * 1000000ULL, WAKE_RF_DISABLED);
}

void publishMsg(const char *t, const char *m, bool r = false) {
  SerPrintf("Publishing to topic: %s, message: ", t);
  SerPrintln(m);
  if (!mqttClient.publish(t, m, r)) {
      SerPrintln("ERROR, Publishing failed");
  };
}

#ifdef EPAPER
void displayBase() {
  display.init();
  display.setFullWindow();
  display.setRotation(1);
  display.fillScreen(GxEPD_WHITE);
}

void displayLowBatt() {
  displayBase();
  display.drawInvertedBitmap(0, 0, gImage_low_batt, 200, 200, GxEPD_BLACK);
  display.display();
}

void displayValues(float temp, int hum, int batt) {
  displayBase();

  display.drawInvertedBitmap(0, 0, gImage_main, 200, 200, GxEPD_BLACK);
  display.setTextColor(GxEPD_WHITE);
  display.setFont(&FreeSansOblique9pt7b);
  if (batt != 100) {
    display.setCursor(162, 24);
  } else {
    display.setCursor(156, 24);
  }  
  display.print(batt);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeSans24pt7b);
  display.setCursor(55, 85);
  display.print(temp,1);
  display.setCursor(95, 172);
  display.print(hum);
  display.display();
}

void displayOff() {
  display.powerOff();
  display.hibernate();
}
#endif

void setup() {
  unsigned long timestamp;
  char client_id[19], topic_base[50], topic_state[70], msg[100];
  int voltage_percentage;
  float voltage;
  rst_info *resetInfo;

  SerBegin(115200);
  SerPrintln("\nStarting measurement iteration");

  SerPrint("Measuring battery voltage: ");
  voltage = ESP.getVcc() / 1024.0f;
  voltage_percentage = MyBattery.calcCr123Percentage(voltage);
  SerPrint(voltage);
  SerPrint(" = ");
  SerPrintln(voltage_percentage);
  if (voltage_percentage == 0) {
  #ifdef EPAPER
    displayLowBatt();
  #endif
    goodnightEsp(0);
  }

  // START BME280
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
    SerPrintln("failed");
    goodnightEsp(SLEEP_TIME_ERROR_SEC);
  }

#if STATIC_IP == 1
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, INADDR_NONE)) {
    SerPrintln("ERROR, failed to configure Wifi");
  }
#endif
  WiFi.begin();

  // BME280 TAKE SINGEL MEASUREMENT, NEED LITTLE PAUSE AFTER INIT
  bme280.setMode(MODE_FORCED);

  //DISPLAY ON EPAPER
  while(bme280.isMeasuring() == true) ; // Wait until measurement is done
  bme280.readAllMeasurements(&measurements, 0);
#ifdef EPAPER
  displayValues(measurements.temperature, (int)measurements.humidity, voltage_percentage);
#endif

  SerPrintln("Waiting for Wifi to connect: ");
  if (WiFi.waitForConnectResult(WIFI_TIMEOUT_SEC*1000) == WL_CONNECTED) {
    SerPrintln("success");
  } else {
    SerPrintln("failed, retrying");

    SerPrint("Starting general wifi connection: ");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    if (WiFi.waitForConnectResult(WIFI_TIMEOUT_SEC*1000) != WL_CONNECTED) {
      SerPrintln("failed, sleeping");
      goodnightEsp(SLEEP_TIME_ERROR_SEC);
    }
    SerPrintln("success");
  }

  ntptime.init();

  // START MQTT
  if (snprintf(client_id, sizeof(client_id), DEVICE_ID, EspChipId.get()) >= (int) sizeof(client_id)) {
    SerPrintln("Mqtt client id cannot be constructed");
  };
  
  BearSSL::X509List *serverTrustedCA = new BearSSL::X509List(ca_cert);
  BearSSL::X509List *serverCertList = new BearSSL::X509List(client_cert);
  BearSSL::PrivateKey *serverPrivKey = new BearSSL::PrivateKey(client_private_key);
  wifiClient.setTrustAnchors(serverTrustedCA);
  wifiClient.setClientRSACert(serverCertList, serverPrivKey);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  if (!mqttClient.setBufferSize(MQTT_BUFFER_SIZE)) {
    SerPrintln("Mqtt buffer cannot be resized");
  }

  if (!ntptime.waitForTime()) {
    SerPrintln("NTP failed");
    goodnightEsp(SLEEP_TIME_ERROR_SEC);
  }

  SerPrint("Starting mqtt connection: ");
  timestamp = millis();
  while (!mqttClient.connected()) {
    if (mqttClient.connect(client_id)) {
      SerPrintln("connected");
    } else {
      SerPrintln("failed, rc=");
      SerPrint(mqttClient.state());
      SerPrintln("");
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
  
  resetInfo = ESP.getResetInfoPtr();
  if (resetInfo->reason != REASON_DEEP_SLEEP_AWAKE) {
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
      char config_doc[400], topic_config[50], uid[20];

      if (strlcpy(topic_config, topic_base, sizeof(topic_config)) >= sizeof(topic_config)) {
        SerPrintln("ERROR, base topic cannot be copyied");
      }
      if (strlcat(topic_config, suffixes[i], sizeof(topic_config)) >= sizeof(topic_config)) {
        SerPrintln("ERROR, config topic suffix cannot be constructed");
      }
      if (strlcat(topic_config, "/config", sizeof(topic_config)) >= sizeof(topic_config)) {
        SerPrintln("ERROR, config topic cannot be constructed");
      }
      int len_sn = snprintf(uid, sizeof(uid), "%08X", EspChipId.get());
      if (len_sn < 0 || (unsigned) len_sn >= (int) sizeof(uid)) {
        SerPrintln("Client id cannot be constructed");
      }
      if (strlcat(uid, suffixes[i], sizeof(uid)) >= sizeof(uid)) {
        SerPrintln("ERROR, uid cannot be constructed");
      }

      StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, configs[i]);
        if (error) {
            SerPrint("ERROR, deserialization failed: ");
            SerPrintln(error.f_str());
            return;
        }
        doc["~"] = topic_base;
        doc["unique_id"] = uid;
        doc["dev"] = serialized(dev_conf);

        if (measureJson(doc) >= sizeof(config_doc)) {
          SerPrintln("ERROR, config document char array too short");
          break;
        }
        serializeJson(doc, config_doc);
        publishMsg(topic_config, config_doc, true);
        // slow down publishing, homeassistant needs some time for registration
        mqttClient.loop();
        delay(500);
    }
  }

  // PUBLISH HASS ATTRIBUTES
  {
    char topic[50];
    if (strlcpy(topic, topic_base, sizeof(topic)) >= sizeof(topic)) {
      SerPrintln("ERROR, base topic cannot be copyied");
    }
    if (strlcat(topic, "/attributes", sizeof(topic)) >= sizeof(topic)) {
          SerPrintln("ERROR, attributes topic cannot be constructed");
    }
    if (snprintf(msg, sizeof(msg), HASS_ATTRIBUTE_COLLECTION, WiFi.RSSI(), voltage) >= (int) sizeof(msg)) {
      SerPrintln("ERROR, attribute collection cannot be constructed");
    }
    publishMsg(topic, msg);
  }
  
  // PUBLISH MEASUREMENTS
  if (snprintf(msg, sizeof(msg), HASS_PAYLOAD_STATE, measurements.temperature, measurements.humidity,
      voltage_percentage) >= (int) sizeof(msg)) {
    SerPrintln("Mqtt state payload cannot be constructed");
  };
  publishMsg(topic_state, msg);

  mqttClient.loop();
  delay(300);  // leave some time for pubsubclient

#ifdef EPAPER
  displayOff();
#endif
  mqttClient.disconnect();

  SerPrintln("All done, sleeping...");
  SerPrintln(millis());
  goodnightEsp(SLEEP_TIME_REGULAR_SEC);
}

void loop() {}