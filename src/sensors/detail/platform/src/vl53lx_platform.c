
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */


#include "vl53lx_platform.h"

#include <vl53lx_platform_log.h>

#define VL53LX_get_register_name(VL53LX_p_007, VL53LX_p_032) VL53LX_COPYSTRING(VL53LX_p_032, "");


#define log(level, ...) _LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_PLATFORM, level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) _LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_NONE, VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)


VL53LX_Error VL53LX_WriteMulti(VL53LX_Dev_t* pdev, uint16_t index, uint8_t* pdata, uint32_t count)
{
}


VL53LX_Error VL53LX_ReadMulti(VL53LX_Dev_t* pdev, uint16_t index, uint8_t* pdata, uint32_t count)
{
}


VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t* pdev, uint16_t index, uint8_t VL53LX_p_003)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint8_t buffer[1];

    buffer[0] = (uint8_t)(VL53LX_p_003);

    status = VL53LX_WriteMulti(pdev, index, buffer, 1);

    return status;
}


VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t* pdev, uint16_t index, uint16_t VL53LX_p_003)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint8_t buffer[2];

    buffer[0] = (uint8_t)(VL53LX_p_003 >> 8);
    buffer[1] = (uint8_t)(VL53LX_p_003 & 0x00FF);

    status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);

    return status;
}

VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t* pdev, uint16_t index, uint32_t VL53LX_p_003)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint8_t buffer[4];

    buffer[0] = (uint8_t)(VL53LX_p_003 >> 24);
    buffer[1] = (uint8_t)((VL53LX_p_003 & 0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((VL53LX_p_003 & 0x0000FF00) >> 8);
    buffer[3] = (uint8_t)(VL53LX_p_003 & 0x000000FF);

    status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);

    return status;
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
	sleep_
}

VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t* pdev, int32_t wait_ms)
{
}

VL53LX_Error VL53LX_GetTickCount(VL53LX_Dev_t* pdev, uint32_t* ptick_count_ms)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    (void)pdev;

    *ptick_count_ms = timeGetTime();

    log(VL53LX_TRACE_LEVEL_DEBUG, "VL53LX_GetTickCount() = %5u ms;\n", *ptick_count_ms);

    return status;
}


VL53LX_Error
    VL53LX_WaitValueMaskEx(VL53LX_Dev_t* pdev, uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask, uint32_t poll_delay_ms)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint32_t start_time_ms = 0;
    uint32_t current_time_ms = 0;
    uint8_t byte_value = 0;
    uint8_t found = 0;
#ifdef VL53LX_LOG_ENABLE
    uint32_t trace_functions = 0;
#endif

    _LOG_STRING_BUFFER(register_name);

    SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53LX_LOG_ENABLE

    VL53LX_get_register_name(index, register_name);


    trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n", timeout_ms, register_name, value, mask, poll_delay_ms);
#endif


    VL53LX_GetTickCount(pdev, &start_time_ms);
    pdev->new_data_ready_poll_duration_ms = 0;


#ifdef VL53LX_LOG_ENABLE
    trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
    _LOG_SET_TRACE_FUNCTIONS(VL53LX_TRACE_FUNCTION_NONE);


    while ((status == VL53LX_ERROR_NONE) && (pdev->new_data_ready_poll_duration_ms < timeout_ms) && (found == 0)) {
        status = VL53LX_RdByte(pdev, index, &byte_value);

        if ((byte_value & mask) == value) {
            found = 1;
        }


        VL53LX_GetTickCount(pdev, &current_time_ms);
        pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
    }


    _LOG_SET_TRACE_FUNCTIONS(trace_functions);

    if (found == 0 && status == VL53LX_ERROR_NONE)
        status = VL53LX_ERROR_TIME_OUT;

    return status;
}
