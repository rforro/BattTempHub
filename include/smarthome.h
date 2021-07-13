#define DEVICE_ID "esp8266-%08X"

#define HASS_BASE_TOPIC "homeassistant/sensor/%s"

#define UID_TEMP_SUFFIX "T"
#define UID_HUM_SUFFIX "H"
#define UID_BATT_SUFFIX "B"

#define HASS_CONF_TEMP "{\"dev_cla\":\"temperature\",\"frc_upd\":\"true\",\"json_attr_t\":\"~/attributes\", \
\"name\":\"Temperature\",\"stat_t\":\"~/state\",\"unit_of_meas\":\"°C\",\"val_tpl\":\"{{value_json.temp}}\", \
\"exp_aft\":\"86400\"}"
#define HASS_CONF_HUM "{\"dev_cla\":\"humidity\",\"frc_upd\":\"true\",\"json_attr_t\":\"~/attributes\", \
\"name\":\"Humidity\",\"stat_t\":\"~/state\",\"unit_of_meas\":\"%\",\"val_tpl\":\"{{value_json.hum}}\", \
\"exp_aft\":\"86400\"}"
#define HASS_CONF_BATT "{\"dev_cla\":\"battery\",\"frc_upd\":\"true\",\"json_attr_t\":\"~/attributes\", \
\"name\":\"Battery\",\"stat_t\":\"~/state\",\"unit_of_meas\":\"%\",\"val_tpl\":\"{{value_json.batt}}\", \
\"exp_aft\":\"86400\"}"

#define HASS_CONF_DEVICE "{\"cns\":[[\"mac\",\"%s\"]],\"ids\":\"%08X\",\"mf\":\"espressif\",\"mdl\":\"ESP8266\",\"name\":\"Battery Thermometer\"}"

#define HASS_ATTRIBUTE_COLLECTION "{\"RSSI\":\"%i\",\"Voltage\":\"%.3f\"}"
#define HASS_PAYLOAD_STATE "{\"temp\":\"%.1f\",\"hum\":\"%.0f\",\"batt\":\"%i\"}"

#define SLEEP_TIME_ERROR_SEC (30*60)
#define SLEEP_TIME_REGULAR_SEC (60*60)

#define WIFI_TIMEOUT_SEC 30
#define MQTT_TIMEOUT_SEC 15
#define MQTT_BUFFER_SIZE 512