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
    VL53L4CX();
    ~VL53L4CX();
    int32_t distance() const override;
};
} // namespace sensors
