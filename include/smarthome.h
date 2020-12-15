#define TOPIC_HASS_CONFIG_TEMP "homeassistant/sensor/%sT/config"
#define TOPIC_HASS_CONFIG_HUM "homeassistant/sensor/%sH/config"
#define TOPIC_HASS_CONFIG_BATT "homeassistant/sensor/%sB/config"
#define TOPIC_HASS_STATE "homeassistant/sensor/%s/state"
#define PAYLOAD_HASS_CONFIG_TEMP "{\"device_class\":\"temperature\",\"name\":\"Temperature\",\"state_topic\":\"%s\"\
,\"unit_of_measurement\":\"°C\",\"value_template\":\"{{ value_json.temperature}}\",\"unique_id\":\"%s-temp\"\
,\"expire_after\":\"86400\",\"json_attr_t\":\"homeassistant/sensor/%s/attributes\"\
,\"device\":{\"name\":\"Smarthaurin-TempHumBatt\",\"model\":\"esp8266\",\"manufacturer\":\"espressif\",\"identifiers\":\"%s\"}\
}"
#define PAYLOAD_HASS_CONFIG_HUM "{\"device_class\":\"humidity\",\"name\":\"Humidity\",\"state_topic\":\"%s\"\
,\"unit_of_measurement\":\"%%\",\"value_template\":\"{{ value_json.bme280_humidity}}\",\"unique_id\":\"%s-hum\"\
,\"expire_after\":\"86400\"\
,\"device\":{\"name\":\"Smarthaurin-TempHumBatt\",\"model\":\"esp8266\",\"manufacturer\":\"espressif\",\"identifiers\":\"%s\"}\
}"
#define PAYLOAD_HASS_CONFIG_BATT "{\"device_class\":\"battery\",\"name\":\"Battery\",\"state_topic\":\"%s\"\
,\"unit_of_measurement\":\"%%\",\"value_template\":\"{{ value_json.battery}}\",\"unique_id\":\"%s-batt\"\
,\"expire_after\":\"86400\"\
,\"device\":{\"name\":\"Smarthaurin-TempHumBatt\",\"model\":\"esp8266\",\"manufacturer\":\"espressif\",\"identifiers\":\"%s\"}\
}"
#define PAYLOAD_HASS_STATE "{\"temperature\": \"%.1f\",\"bme280_humidity\":\"%.0f\",\"battery\":\"%i\"}"

#define PAYLOAD_HASS_ATTR "{\"IP\":\"192.168.1.1\",\"RSSI\": \"-75\"}"

#define BATTERY_CR123A_HIGH 3.0
#define BATTERY_CR123A_LOW 2.0

#define SLEEP_TIME_ERROR_SEC (30*60)
#define SLEEP_TIME_REGULAR_SEC (10*60)

#define WIFI_TIMEOUT_SEC 30
#define MQTT_TIMEOUT_SEC 15
#define MQTT_BUFFER_SIZE 512