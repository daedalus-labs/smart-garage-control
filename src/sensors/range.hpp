/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#pragma once

#include <cstdint>
#include <string_view>


namespace sensors {
/**
 * A generic controller for a Heating element.
 */
class Range
{
public:
    virtual ~Range() = default;

    /**
     * @return The current range read by the sensor in millimeters.
     */
    virtual int32_t distance() const = 0;
};
} // namespace controllers
