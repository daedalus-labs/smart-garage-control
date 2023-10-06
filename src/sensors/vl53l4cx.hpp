/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#pragma once

#include "sensors/range.hpp"
#include "vl53lx_api.h"

#include <cstdint>
#include <string_view>


namespace sensors {
class VL53L4CX final : public Range
{
public:
    VL53L4CX(uint8_t address, uint8_t shutdown_pin, size_t retry_count);
    ~VL53L4CX();
    int32_t distance() const override;
    bool start();

private:
    void _off();
    void _on();

    bool _verify() const;

    const uint8_t _shutdown_pin;
    const size_t _max_retries;
    mutable VL53LX_Dev_t _device;
};
} // namespace sensors
