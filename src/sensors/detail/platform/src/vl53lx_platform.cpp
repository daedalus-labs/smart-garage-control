
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */


#include "vl53lx_platform.h"

#include "utilities.hpp"

#include <hardware/i2c.h>
#include <hardware/timer.h>
#include <pico/time.h>
#include <vl53lx_platform_log.h>


#define log(level, ...) _LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_PLATFORM, level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) _LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_NONE, VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)


VL53LX_Error VL53LX_WriteMulti(VL53LX_Dev_t* pdev, uint16_t register_address, uint8_t* pdata, uint32_t count)
{
    uint8_t buffer_size = 3 * count;
    uint8_t buffer[buffer_size];
    size_t index = 0, offset = 0;

    for (index = 0, offset = 0; offset < count; index += 3, offset++) {
        buffer[index] = static_cast<uint8_t>((register_address + index) >> 8);
        buffer[index + 1] = static_cast<uint8_t>((register_address + index) & 0xFF);
        buffer[index + 2] = *(pdata + offset);
    }

    int32_t write_count = i2c_write_blocking(i2c_default, pdev->i2c_slave_address, buffer, buffer_size, false);

    if (write_count != buffer_size) {
        fprintf(stderr, "Failed to write to [%u, %u]: %d", pdev->i2c_slave_address, register_address, write_count);
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_ReadMulti(VL53LX_Dev_t* pdev, uint16_t register_address, uint8_t* pdata, uint32_t count)
{
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(register_address >> 8);
    buffer[1] = static_cast<uint8_t>(register_address & 0xFF);
    i2c_write_blocking(i2c_default, pdev->i2c_slave_address, buffer, sizeof(buffer), true);

    int32_t read_count = i2c_read_blocking(i2c_default, pdev->i2c_slave_address, pdata, count, false);

    if (read_count < 0 || static_cast<uint32_t>(read_count) != count) {
        fprintf(stderr, "Failed to read from [%u, %u]: %d\n", pdev->i2c_slave_address, register_address, read_count);
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t* pdev, uint16_t index, uint8_t data)
{
    uint8_t buffer[1];
    buffer[0] = static_cast<uint8_t>(data);

    return VL53LX_WriteMulti(pdev, index, buffer, 1);
}

VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t* pdev, uint16_t index, uint16_t data)
{
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(data >> 8);
    buffer[1] = static_cast<uint8_t>(data & 0x00FF);

    return VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);
}

VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t* pdev, uint16_t index, uint32_t data)
{
    uint8_t buffer[4];
    buffer[0] = static_cast<uint8_t>(data >> 24);
    buffer[1] = static_cast<uint8_t>((data & 0x00FF0000) >> 16);
    buffer[2] = static_cast<uint8_t>((data & 0x0000FF00) >> 8);
    buffer[3] = static_cast<uint8_t>(data & 0x000000FF);

    return VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);
}

VL53LX_Error VL53LX_RdByte(VL53LX_Dev_t* pdev, uint16_t index, uint8_t* pdata)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint8_t buffer[1];

    status = VL53LX_ReadMulti(pdev, index, buffer, 1);

    *pdata = buffer[0];

    return status;
}

VL53LX_Error VL53LX_RdWord(VL53LX_Dev_t* pdev, uint16_t index, uint16_t* pdata)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint8_t buffer[2];

    status = VL53LX_ReadMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);

    *pdata = (uint16_t)(((uint16_t)(buffer[0]) << 8) + (uint16_t)buffer[1]);

    return status;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_Dev_t* pdev, uint16_t index, uint32_t* pdata)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint8_t buffer[4];

    status = VL53LX_ReadMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);

    *pdata = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];

    return status;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t* pdev, int32_t wait_us)
{
    (void)pdev;
    sleep_us(wait_us);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t* pdev, int32_t wait_ms)
{
    (void)pdev;
    sleep_ms(wait_ms);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTickCount(VL53LX_Dev_t* pdev, uint32_t* ptick_count_ms)
{
    (void)pdev;
    *ptick_count_ms = static_cast<uint32_t>(milliseconds());
    return VL53LX_ERROR_NONE;
}


VL53LX_Error
    VL53LX_WaitValueMaskEx(VL53LX_Dev_t* pdev, uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask, uint32_t poll_delay_ms)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint64_t start_time = milliseconds();
    uint64_t polling_time = 0;
    uint8_t read_value;
    bool found = false;

    while ((status == VL53LX_ERROR_NONE) && (polling_time < timeout_ms) && !found) {
        if (status == VL53LX_ERROR_NONE) {
            status = VL53LX_RdByte(pdev, index, &read_value);
        }

        found = (read_value & mask) == value;

        if (status == VL53LX_ERROR_NONE && !found && poll_delay_ms > 0) {
            sleep_ms(poll_delay_ms);
            polling_time = milliseconds() - start_time;
        }
    }

    if (!found && status == VL53LX_ERROR_NONE) {
        return VL53LX_ERROR_TIME_OUT;
    }

    return status;
}
