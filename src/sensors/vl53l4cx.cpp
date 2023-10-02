/*------------------------------------------------------------------------------
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause

This code uses reference code copyright 2022 by STMicroelectronics
------------------------------------------------------------------------------*/

#include "vl53l4cx.hpp"

#include "gpio.hpp"

#include <hardware/gpio.h>
#include <hardware/i2c.h>

#include <cstdint>

/**
 * Boot time in the datasheet is defined as 1.2, using 2 here to avoid attempting communication before boot has finished.
 * @see docs/Datasheet-VL53L4CX
 */
inline constexpr uint32_t T_BOOT_MS = 3;

/** The Model ID and Module Type registers are used to validate communication. */
inline constexpr uint16_t MODEL_ID_INDEX = 0x010F;
inline constexpr uint16_t MODULE_TYPE_INDEX = 0x0110;
inline constexpr uint8_t MODEL_ID_EXPECTED_VALUE = 0xEB;
inline constexpr uint8_t MODULE_TYPE_EXPECTED_VALUE = 0xAA;


namespace sensors {
VL53L4CX::VL53L4CX(uint8_t address, uint8_t shutdown_pin) : _address(address), _shutdown_pin(shutdown_pin)
{
    gpio_init(_shutdown_pin);
    gpio_set_dir(_shutdown_pin, GPIO_OUT);

    // Turning the sensor off first to avoid any issues caused by
    // the floating GPIO.
    _off();
    _on();
}

VL53L4CX::~VL53L4CX()
{
    _off();
}

int32_t VL53L4CX::distance() const
{
    return 0;
}

bool VL53L4CX::start()
{}

bool VL53L4CX::_checkDeviceInitialized()
{}

bool VL53L4CX::_initializeData()
{}

bool VL53L4CX::_setDeviceAddress()
{}

void VL53L4CX::_off()
{
    gpio_put(_shutdown_pin, OFF);
    sleep_ms(T_BOOT_MS);
}

void VL53L4CX::_on()
{
    gpio_put(_shutdown_pin, ON);
    sleep_ms(T_BOOT_MS);
}
} // namespace sensors
