/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause
------------------------------------------------------------------------------*/
#pragma once

#include <cstdint>
#include <string_view>


/** Enumeration of the possible status values of a Wireless Connection */
enum class DoorStatus : uint8_t
{
    OPEN = 0,    ///< The door is open
    OPENING = 1, ///< The door is transitioning from close to open
    CLOSING = 2, ///< The door is transitioning from open to close
    CLOSED = 3   ///< The door is closed
};

/**
 * Checks if @a status is a transient state or not.
 *
 * @note A transient state is one that indicates the door is moving.
 * @param[in] status The DoorStatus value to check.
 * @return True if @a status is a transient statue, false otherwise.
 */
bool isTransient(DoorStatus status);

/**
 * Converts @a status to a human readable string.
 *
 * @param[in] status The DoorStatus value.
 * @return A human readable name for @a status.
 */
std::string_view toString(DoorStatus status);
