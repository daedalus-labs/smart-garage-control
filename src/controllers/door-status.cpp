/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#include "door-status.hpp"

#include <string_view>

inline constexpr std::string_view OPEN = "open";
inline constexpr std::string_view CLOSED = "closed";
inline constexpr std::string_view OPENING = "opening";
inline constexpr std::string_view CLOSING = "closing";


bool isTransient(DoorStatus status)
{
    return (status == DoorStatus::CLOSING || status == DoorStatus::OPENING);
}

std::string_view toString(DoorStatus status)
{
    switch (status) {
    case DoorStatus::CLOSED:
        return CLOSED;
    case DoorStatus::OPENING:
        return OPENING;
    case DoorStatus::CLOSING:
        return CLOSING;
    case DoorStatus::OPEN:
    default:
        return OPEN;
    }
}
