#ifndef SmarthaurinHassEmbed_h
#define SmarthaurinHassEmbed_h

#include <Arduino.h>

#define PAYLOAD_HASS_CONFIG_BASE "{\"unique_id\":\"%s\",\
\"device\":{\"name\":\"Smarthaurin-TempHumBatt\",\"model\":\"esp8266\",\"manufacturer\":\"espressif\",\"identifiers\":\"%s\"}\
}"

#define TOPIC_HASS_DISCOVERY "homeassistant/%s/%s/%s"

class SmarthaurinHassEmbed
{
private:
    const char* baseConfig;
    char chip_id[20];
    uint32_t espGetChipId();

public:
    SmarthaurinHassEmbed();
    ~SmarthaurinHassEmbed();

    boolean getConfigBase(const char* t);
    boolean getTopicSensorState(char* const topic, size_t length);
};

#endif