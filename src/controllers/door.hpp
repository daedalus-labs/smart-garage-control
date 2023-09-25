/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#pragma once

#include "door-status.hpp"

#include <cstdint>
#include <string_view>


namespace controllers {
/**
 * A generic controller for a Heating element.
 */
class Door
{
public:
    /**
     * Constructor.
     *
     * @param[in] door_number The door number.
     * @param[in] control_pin The pin assigned to the Door Relay.
     * @param[in] sensor_pin The pin assigned to the Door Feedback sensor.
     */
    Door(uint8_t door_number, uint8_t control_pin, uint8_t sensor_pin);

    /**
     * Sets the target temperature for the Door.
     *
     * @param[in] desired_status The desired target temperature in degrees Celsius.
     */
    void set(DoorStatus desired_status);

    /**
     * @return The current status of the door.
     */
    DoorStatus status();

private:
    /**
     * Toggles the relay controlling the door.
     */
    void _toggle();

    /**
     * Updates all internal state information
     */
    void _update();

    const uint8_t _sensor_pin;
    const uint8_t _control_pin;
    const uint8_t _door_number;
    DoorStatus _current_status;
    DoorStatus _desired_status;
};
} // namespace controllers
