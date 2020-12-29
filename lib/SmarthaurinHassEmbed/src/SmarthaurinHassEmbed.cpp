#include "SmarthaurinHassEmbed.h"
#include "Arduino.h"

SmarthaurinHassEmbed::SmarthaurinHassEmbed() {
    this->baseConfig = PAYLOAD_HASS_CONFIG_BASE;
    snprintf(this->chip_id, sizeof(this->chip_id), "esp-%08X", this->espGetChipId()) >= (int) sizeof(this->chip_id);
}

boolean SmarthaurinHassEmbed::getConfigBase(const char *t)
{

}

boolean SmarthaurinHassEmbed::getTopicSensorState(char* const topic, size_t length) {
    return snprintf(topic, length, TOPIC_HASS_DISCOVERY, "sensor", this->chip_id, "state") >= (int) length;
}

uint32_t SmarthaurinHassEmbed::espGetChipId() {
#if defined(ESP8266)
    return ESP.getChipId();
#elif defined(ESP32)
    uint32_t chip_id = 0;
    for (uint8_t i = 0; i < 25; i = i + 8) {
        chip_id |= ((ESP.getEfuseMac() >> (40u - i)) & 0xffu) << i;
    }
    return chip_id;
#endif
}