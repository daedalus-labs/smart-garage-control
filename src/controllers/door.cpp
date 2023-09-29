/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/

#include "door.hpp"

#include "door-status.hpp"
#include "gpio.hpp"
#include "sensors/vl53l4cx.hpp"

#include <hardware/gpio.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>

#include <cstdint>
#include <cstdio>
#include <string_view>


inline constexpr uint32_t TOGGLE_BOUNCE_MS = 150;

namespace controllers {
Door::Door(uint8_t door_number, uint8_t control_pin)
    : _control_pin(control_pin),
      _door_number(door_number),
      _current_status(DoorStatus::OPEN),
      _desired_status(DoorStatus::OPEN),
      _range_sensor(std::make_unique<sensors::VL53L4CX>())
{
    gpio_init(_control_pin);
    gpio_set_dir(_control_pin, GPIO_OUT);
    _update();
}

int32_t Door::closeDistance() const
{
    return _range_sensor->distance();
}

uint8_t Door::number() const
{
    return _door_number;
}

void Door::set(DoorStatus desired_status)
{
    if (desired_status == _desired_status) {
        printf("Cannot set door %u: Already %s\n", _door_number, toString(_desired_status));
        return;
    }

    if (isTransient(_desired_status)) {
        fprintf(stderr, "Cannot set door %u: Current state (%s) is transient", _door_number, toString(_desired_status));
        return;
    }

    if (isTransient(desired_status)) {
        fprintf(stderr, "Cannot set door %u: Desired state (%s) is transient", _door_number, toString(_desired_status));
        return;
    }

    _desired_status = desired_status;

    if (_desired_status == DoorStatus::CLOSED) {
        _current_status = DoorStatus::CLOSING;
    }
    else {
        _current_status = DoorStatus::OPENING;
    }

    _toggle();
    _update();
}

DoorStatus Door::status()
{
    return _current_status;
}

void Door::_toggle()
{
    gpio_put(_control_pin, ON);
    sleep_ms(TOGGLE_BOUNCE_MS);
    gpio_put(_control_pin, OFF);
    sleep_ms(TOGGLE_BOUNCE_MS);
}

void Door::_update()
{}
} // namespace controllers
