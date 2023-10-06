/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#pragma once

#include "door-status.hpp"
#include "sensors/range.hpp"
#include "utilities.hpp"

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
     * @param[in] cfg The System Configuration.
     */
    Door(const SystemConfiguration &cfg);

    /**
     * @return The current door closure distance in millimeters.
     */
    int32_t closeDistance() const;

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
    DoorStatus _current_status;
    DoorStatus _desired_status;
    std::unique_ptr<sensors::Range> _range_sensor;
};
} // namespace controllers
