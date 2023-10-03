/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#include "controllers/door.hpp"
#include "generated/configuration.hpp"
#include "gpio.hpp"
#include "mqtt.hpp"
#include "utilities.hpp"

#include <hardware/adc.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/multicore.h>
#include <pico/stdio.h>
#include <pico/util/queue.h>
#include <pico/binary_info.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

inline constexpr uint32_t COMMUNICATION_PERIOD_MS = 10000;
inline constexpr uint32_t MQTT_CONNECTION_WAIT_MS = 17500;
inline constexpr uint32_t INITIALIZATION_WAIT_MS = 5000;
inline constexpr uint32_t DATA_PERIOD_MS = 5000;
inline constexpr uint32_t I2C_BAUDRATE_HZ = 400000;
inline constexpr uint8_t QUEUE_SIZE = 10;

typedef struct
{
    float board_temperature;
    size_t door_number;
    int32_t current_close_distance;
    char current_status[controllers::DOOR_STATUS_STRING_LENGTH];
} feedback_entry;

typedef struct
{
    size_t door_number;
    char desired_status[controllers::DOOR_STATUS_STRING_LENGTH];
    ;
} request_entry;

queue_t feedback_queue;
queue_t request_queue;
SystemConfiguration configuration;

void processRequest(const request_entry& request, std::vector<controllers::Door>& doors)
{
    if (request.door_number > doors.size()) {
        fprintf(stderr, "Invalid request to set door %u to %s: Door does not exist\n", request.door_number, request.desired_status);
        return;
    }

    controllers::DoorStatus desired_status;
    if (!controllers::parse(request.desired_status, desired_status)) {
        fprintf(stderr,
                "Invalid request to set door %u to %s: %s is not a valid status\n",
                request.door_number,
                request.desired_status,
                request.desired_status);
        return;
    }

    doors[request.door_number].set(desired_status);
}

void controlLoop()
{
    std::vector<controllers::Door> doors;
    for (size_t i = 0; i < configuration.doorPins().size(); i++) {
        controllers::Door new_door(i, configuration.doorPins()[i]);
        doors.push_back(std::move(new_door));
    }

    while (true) {
        if (!queue_is_empty(&request_queue)) {
            request_entry request;
            queue_remove_blocking(&request_queue, &request);
            processRequest(request, doors);
        }

        for (auto& door : doors) {
            feedback_entry door_feedback;
            std::memset(door_feedback.current_status, 0, sizeof(door_feedback.current_status));
            std::strcpy(door_feedback.current_status, controllers::toString(door.status()).data());
            door_feedback.current_close_distance = door.closeDistance();
            door_feedback.door_number = door.number();
            queue_add_blocking(&feedback_queue, &door_feedback);
        }
        sleep_ms(DATA_PERIOD_MS);
    }
}

void publish(mqtt::Client& client, const feedback_entry& data)
{
    std::string distance = std::to_string(data.current_close_distance);
    std::string board_temperature = std::to_string(data.board_temperature);

    char mqtt_topic[TOPIC_BUFFER_SIZE];
    snprintf(mqtt_topic, TOPIC_BUFFER_SIZE, BOARD_TEMPERATURE_TOPIC_FORMAT.data(), client.deviceName().c_str());
    mqtt::publish(client, mqtt_topic, board_temperature, true);

    snprintf(mqtt_topic, TOPIC_BUFFER_SIZE, DOOR_STATE_TOPIC_FORMAT.data(), client.deviceName().c_str(), data.door_number);
    mqtt::publish(client, mqtt_topic, data.current_status, true);

    snprintf(mqtt_topic, TOPIC_BUFFER_SIZE, DOOR_CLOSE_DISTANCE_TOPIC_FORMAT.data(), client.deviceName().c_str(), data.door_number);
    mqtt::publish(client, mqtt_topic, distance, true);
}

static void onDoorSetState(const std::string& topic, const mqtt::Buffer& data)
{
    request_entry set_request;
    std::string value = mqtt::toString(data);
    std::memset(set_request.desired_status, 0, sizeof(set_request.desired_status));
    std::strcpy(set_request.desired_status, value.c_str());

    /** @todo What is the best way to get door number? */

    printf("Received request to set door %u to %s\n", set_request.door_number, set_request.desired_status);
    queue_add_blocking(&request_queue, &set_request);
}

static void initialize()
{
    stdio_init_all();
    adc_init();

    queue_init(&feedback_queue, sizeof(feedback_entry), QUEUE_SIZE);
    queue_init(&request_queue, sizeof(request_entry), QUEUE_SIZE);

    gpio_init(SYSTEM_LED_PIN);
    gpio_set_dir(SYSTEM_LED_PIN, GPIO_OUT);
    gpio_put(SYSTEM_LED_PIN, ON);
}

static void initializeI2C()
{
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #error Please provide an environment with a default i2c interface
#else
    // This project will use `i2c_default` as the global i2c bus.
    i2c_init(i2c_default, I2C_BAUDRATE_HZ);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
#endif
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

    for (size_t i = 0; i < configuration.doorPins().size(); i++) {
        snprintf(mqtt_topic, TOPIC_BUFFER_SIZE, SET_DOOR_STATE_TOPIC_FORMAT.data(), client.deviceName().c_str(), i);
        if (!client.subscribe(mqtt_topic, onDoorSetState)) {
            printf("Failed to subscribe to %s\n", mqtt_topic);
            return false;
        }
    }

    printf("Successfully initialized MQTT\n");
    return true;
}

int main(int argc, char** argv)
{
    initialize();
    initializeI2C();
    std::string board_id = systemIdentifier();
    bool mqtt_initialized = false;
    uint32_t count = 0;

    sleep_ms(INITIALIZATION_WAIT_MS);
    if (!read(configuration)) {
        printf("Failed to read system configuration\n");
        return EXIT_FAILURE;
    }

    mqtt::Client client(configuration.mqttBroker(), CONFIGURED_MQTT_PORT, configuration.deviceName(), MQTT_FEEDBACK_PIN);
    sleep_ms(COMMUNICATION_PERIOD_MS);

    multicore_launch_core1(controlLoop);

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
