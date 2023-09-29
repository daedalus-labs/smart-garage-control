/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#include "door-status.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>

inline constexpr std::string_view OPEN = "open";
inline constexpr std::string_view CLOSED = "closed";
inline constexpr std::string_view OPENING = "opening";
inline constexpr std::string_view CLOSING = "closing";


namespace controllers {
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

bool parse(std::string string_in, DoorStatus& status_out)
{
    // Convert the string to lower case to match the standard values.
    std::transform(string_in.begin(), string_in.end(), string_in.begin(), [](unsigned char c) { return std::tolower(c); });

    if (string_in == OPEN) {
        status_out = DoorStatus::OPEN;
        return true;
    }

    if (string_in == CLOSED) {
        status_out = DoorStatus::CLOSED;
        return true;
    }

    if (string_in == CLOSING) {
        status_out = DoorStatus::CLOSING;
        return true;
    }

    if (string_in == OPENING) {
        status_out = DoorStatus::OPENING;
        return true;
    }

    return false;
}
} // namespace controllers
