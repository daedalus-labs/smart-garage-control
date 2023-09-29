/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/

#include "vl53l4cx.hpp"

namespace sensors {
VL53L4CX::VL53L4CX()
{}

VL53L4CX::~VL53L4CX() = default;

int32_t VL53L4CX::distance() const
{
    return 0;
}
} // namespace sensors
