/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#pragma once

#include "sensors/range.hpp"

#include <cstdint>
#include <string_view>


namespace sensors {
class VL53L4CX final : public Range
{
public:
    VL53L4CX(uint8_t address, uint8_t shutdown_pin);
    ~VL53L4CX();
    int32_t distance() const override;
    bool start();

private:
    bool _checkDeviceInitialized();
    bool _initializeData();
    bool _setDeviceAddress();

    void _off();
    void _on();

    const uint8_t _address;
    const uint8_t _shutdown_pin;
};
} // namespace sensors
