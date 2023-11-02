/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause

This code uses reference code copyright 2022 by STMicroelectronics
------------------------------------------------------------------------------*/

#include "vl53l4cx.hpp"

#include "gpio.hpp"
#include "vl53l5cx_api.h"

#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/stdio.h>

#include <cstdint>
#include <cstdio>

/**
 * Boot time in the datasheet is defined as 1.2, using 3 here to be extra cautious.
 * @see docs/Datasheet-VL53L4CX
 */
inline constexpr uint32_t T_BOOT_MS = 3;
inline constexpr uint32_t POLL_INTERVAL_MS = 5;
inline constexpr uint8_t DEFAULT_VL53L4CX_ADDRESS = 0x54;

/** The Model ID and Module Type registers are used to validate communication. */
inline constexpr uint16_t MODEL_ID_INDEX = 0x010F;
inline constexpr uint16_t MODULE_TYPE_INDEX = 0x0110;
inline constexpr uint8_t MODEL_ID_EXPECTED_VALUE = 0xEB;
inline constexpr uint8_t MODULE_TYPE_EXPECTED_VALUE = 0xAA;

inline constexpr int32_t DISTANCE_ERROR = INT32_MIN;


namespace sensors {
VL53L4CX::VL53L4CX(uint8_t address, uint8_t shutdown_pin, size_t retry_count)
    : _shutdown_pin(shutdown_pin), _max_retries(retry_count), _device()
{
    _device.platform.address = address;
    _device.platform.i2c = i2c_default;
    gpio_init(_shutdown_pin);
    gpio_set_dir(_shutdown_pin, GPIO_OUT);

    // Turning the sensor off first to avoid any issues caused by
    // the floating GPIO.
    _off();
    _on();

    printf("VL53L4CX initialized [i2c Address %u, XSHUT %u]\n", _device.platform.address, _shutdown_pin);
}

VL53L4CX::~VL53L4CX()
{
    stop();
    sleep_ms(T_BOOT_MS);
    _off();
}

int32_t VL53L4CX::distance() const
{
    uint8_t data_ready;
    VL53LX_Error status;
    size_t retry_count;
    VL53LX_MultiRangingData_t data;

    for (retry_count = 0; retry_count < _max_retries; retry_count++) {
        status = VL53LX_GetMeasurementDataReady(&_device, &data_ready);
        if (status == VL53LX_ERROR_NONE && data_ready == VL53LX_ERROR_NONE) {
            break;
        }
        sleep_ms(POLL_INTERVAL_MS);
    }

    if (retry_count >= _max_retries) {
        fprintf(stderr, "VL53LX is not ready: [Retry Count: %u, Last Status: %u, Last Data Ready: %u]\n", retry_count, status, data_ready);
        return DISTANCE_ERROR;
    }

    status = VL53LX_GetMultiRangingData(&_device, &data);
    if (status != VL53LX_ERROR_NONE || data.RangeData->RangeStatus != VL53LX_ERROR_NONE) {
        fprintf(stderr, "Failed to get range data from VL53LX (%u, %u)\n", status, data.RangeData->RangeStatus);
        return DISTANCE_ERROR;
    }
    return data.RangeData->RangeMilliMeter;
}

bool VL53L4CX::start()
{
    if (!_verify()) {
        fprintf(stderr, "VL53L4CX failed verification\n");
        return false;
    }

    VL53LX_WaitDeviceBooted(&_device);
    VL53LX_DataInit(&_device);
    if (VL53LX_StartMeasurement(&_device) != VL53LX_ERROR_NONE) {
        fprintf(stderr, "VL53L4CX failed to start\n");
        return false;
    }
    printf("VL53L4CX started\n");
    return true;
}

bool VL53L4CX::stop()
{
    if(vl53l5cx_stop_ranging(&_device) == VL53L5CX_OK) {
        return true;
    }
    return false;
}

void VL53L4CX::_off()
{
    gpio_put(_shutdown_pin, HIGH);
    sleep_ms(T_BOOT_MS);
}

void VL53L4CX::_on()
{
    gpio_put(_shutdown_pin, LOW);
    sleep_ms(T_BOOT_MS);
}

bool VL53L4CX::_is_alive() const
{
    uint8_t is_alive;
    return vl53l5cx_is_alive() == VL53L5CX_OK && is_alive == VL53L5CX_ALIVE;
}

bool VL53L4CX::_verify() const
{
    uint8_t model_id, model_type;

    RdByte(&_device, MODEL_ID_INDEX, &model_id);
    RdByte(&_device, MODULE_TYPE_INDEX, &model_type);
    printf("VL53L4CX @ %u has Model ID 0x%02X and Model Type 0x%02X\n", _device.i2c_slave_address, model_id, model_type);

    return model_id == MODEL_ID_EXPECTED_VALUE && model_type == MODULE_TYPE_EXPECTED_VALUE;
}
} // namespace sensors
