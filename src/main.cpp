/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#include "generated/configuration.hpp"
#include "gpio.hpp"
#include "mqtt.hpp"
#include "utilities.hpp"

#include <hardware/adc.h>
#include <hardware/gpio.h>
#include <pico/stdio.h>

#include <cstdint>
#include <string>

inline constexpr uint32_t COMMUNICATION_PERIOD_MS = 10000;
inline constexpr uint32_t MQTT_CONNECTION_WAIT_MS = 17500;

static void initialize()
{
    stdio_init_all();
    adc_init();

    gpio_init(SYSTEM_LED_PIN);
    gpio_set_dir(SYSTEM_LED_PIN, GPIO_OUT);
    gpio_put(SYSTEM_LED_PIN, ON);
}

static bool initializeMQTT(mqtt::Client& client, const std::string& uid)
{
    char mqtt_topic[TOPIC_BUFFER_SIZE];
    snprintf(mqtt_topic, TOPIC_BUFFER_SIZE, VERSION_TOPIC_FORMAT.data(), client.deviceName().c_str());
    if (!client.publish(mqtt_topic, static_cast<const void*>(VERSION.data()), VERSION.size(), mqtt::QoS::EXACTLY_ONCE, true)) {
        printf("Failed to publish %s\n", mqtt_topic);
        return false;
    }

    snprintf(mqtt_topic, TOPIC_BUFFER_SIZE, UID_TOPIC_FORMAT.data(), client.deviceName().c_str());
    if (!client.publish(mqtt_topic, static_cast<const void*>(uid.c_str()), uid.size(), mqtt::QoS::EXACTLY_ONCE, true)) {
        printf("Failed to publish %s\n", mqtt_topic);
        return false;
    }

    printf("Successfully initialized MQTT\n");
    return true;
}

int main(int argc, char** argv)
{
    initialize();
    std::string board_id = systemIdentifier();
    bool mqtt_initialized = false;
    uint32_t count = 0;

    sleep_ms(5000);
    SystemConfiguration cfg;
    if (!read(cfg)) {
        printf("Failed to read system configuration\n");
        return EXIT_FAILURE;
    }

    mqtt::Client client(cfg.mqttBroker(), CONFIGURED_MQTT_PORT, cfg.deviceName(), MQTT_FEEDBACK_PIN);

    while (true) {
        if (!client.connected()) {
            printf("Connecting MQTT...\n");
            mqtt_initialized = false;
            client.connect();
            sleep_ms(MQTT_CONNECTION_WAIT_MS);
            continue;
        }

        if (!mqtt_initialized) {
            printf("Initializing MQTT...\n");
            mqtt_initialized = initializeMQTT(client, board_id);
            sleep_ms(COMMUNICATION_PERIOD_MS);
            continue;
        }

        printf("\n----------------- [%u]\n", count);
        printf("MQTT Status: %s\n", client.connected() ? "true" : "false");
        printf("-----------------\n");

        sleep_ms(COMMUNICATION_PERIOD_MS);
        count++;
    }
}
