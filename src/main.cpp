/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#include "generated/configuration.hpp"
#include "gpio.hpp"

#include "mqtt.hpp"

#include <hardware/gpio.h>

int main(int argc, char** argv)
{
    gpio_init(SYSTEM_LED_PIN);
    gpio_set_dir(SYSTEM_LED_PIN, GPIO_OUT);
    gpio_put(SYSTEM_LED_PIN, ON);

    mqtt::Client mqtt("Fomalhaut", CONFIGURED_MQTT_PORT, "test", MQTT_FEEDBACK_PIN);
}
