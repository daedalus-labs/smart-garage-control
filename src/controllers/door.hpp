/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#pragma once

#include "door-status.hpp"
#include "sensors/range.hpp"

#include <cstdint>
#include <memory>
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
     */
    Door(uint8_t door_number, uint8_t control_pin);

    /**
     * @return The number identifying the door.
     */
    uint8_t number() const;

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

    const uint8_t _control_pin;
    const uint8_t _door_number;
    DoorStatus _current_status;
    DoorStatus _desired_status;
    std::unique_ptr<sensors::Range> _range_sensor;
};
} // namespace controllers
