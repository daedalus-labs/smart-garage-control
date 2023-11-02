/*------------------------------------------------------------------------------
Copyright (c) 2020 STMicroelectronics
Copyright (c) 2023 Joe Porembski
SPDX-License-Identifier: BSD-3-Clause

Based on code provided by Pimoroni
------------------------------------------------------------------------------*/

#include "platform.h"

#include <pico/stdlib.h>


uint8_t RdByte(VL53L5CX_Platform* platform, uint16_t address, uint8_t* p_value)
{
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>(address >> 8);
    buf[1] = static_cast<uint8_t>(address & 0xFF);

    i2c_write_blocking(platform->i2c, platform->address, buf, sizeof(buf), true);
    if (i2c_read_blocking(platform->i2c, platform->address, p_value, 1, false) != PICO_ERROR_GENERIC) {
        return VL53L5CX_OK;
    }
    return VL53L5CX_FAIL;
}

uint8_t WrByte(VL53L5CX_Platform* platform, uint16_t address, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>(address >> 8);
    buf[1] = static_cast<uint8_t>(address & 0xFF);
    buf[2] = value;

    if (i2c_write_blocking(platform->i2c, platform->address, buf, sizeof(buf), false) != PICO_ERROR_GENERIC) {
        return VL53L5CX_OK;
    }
    return VL53L5CX_FAIL;
}

uint8_t WrMulti(VL53L5CX_Platform* platform, uint16_t address, uint8_t* values, uint32_t size)
{
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>(address >> 8);
    buf[1] = static_cast<uint8_t>(address & 0xFF);

    // Send the 16-bit address with no STOP condition
    // Handle an error early... it gets dicey from here
    if (i2c_write_blocking(platform->i2c, platform->address, buf, sizeof(buf), true) == PICO_ERROR_GENERIC) {
        return VL53L5CX_FAIL;
    }

    // The VL53L5CX does not support "Repeated Start" and the Pico's I2C API doesn't
    // let us send more bytes without sending another start condition.
    // The horrow below lets us send out "values" followed by a STOP condition,
    // without having to copy everything into a temporary buffer.
    uint8_t* src = values;

    // Send the rest of the data, followed by a STOP condition
    // This re-implements the relevant portion of i2c_write_blocking_internal which is NOT sent a timeout check function by
    // i2c_write_blocking
    for (int32_t byte_ctr = 0; byte_ctr < size; ++byte_ctr) {
        bool last = byte_ctr == (size - 1);
        platform->i2c->hw->data_cmd = bool_to_bit(last) << I2C_IC_DATA_CMD_STOP_LSB | *src++;

        // Wait until the transmission of the address/data from the internal
        // shift register has completed. For this to function correctly, the
        // TX_EMPTY_CTRL flag in IC_CON must be set. The TX_EMPTY_CTRL flag
        // was set in i2c_init.
        do {
            tight_loop_contents();
        } while (!(platform->i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS));

        if (platform->i2c->hw->tx_abrt_source) {
            // Note clearing the abort flag also clears the reason, and
            // this instance of flag is clear-on-read! Note also the
            // IC_CLR_TX_ABRT register always reads as VL53L5CX_OK.
            platform->i2c->hw->clr_tx_abrt;

            // An abort on the LAST byte means things are probably fine
            if (last) {
                // TODO Could there be an abort while waiting for the STOP
                // condition here? If so, additional code would be needed here
                // to take care of the abort.
                do {
                    tight_loop_contents();
                } while (!(platform->i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS));
            }
            else {
                // Ooof, unhandled abort. Fail?
                return VL53L5CX_FAIL;
            }
        }
    }

    // Not sure it matters where we clear this, but by default a "nostop" style write
    // will set this flag so the next transaction starts with a "Repeated Start."
    platform->i2c->restart_on_next = false;
    return VL53L5CX_OK;
}

uint8_t RdMulti(VL53L5CX_Platform* platform, uint16_t address, uint8_t* values, uint32_t size)
{
    const uint8_t buf[2] = {address >> 8, address & 0xFF};
    i2c_write_blocking(platform->i2c, platform->address, buf, sizeof(buf), true);
    if (i2c_read_blocking(platform->i2c, platform->address, values, size, false) != PICO_ERROR_GENERIC) {
        return VL53L5CX_OK;
    }
    return VL53L5CX_FAIL;
}

void SwapBuffer(uint8_t* buffer, uint16_t size)
{
    uint32_t i, tmp;
    /* Example of possible implementation using <string.h> */
    for (i = VL53L5CX_OK; i < size; i = i + 4) {
        tmp = (buffer[i] << 24) | (buffer[i + 1] << 16) | (buffer[i + 2] << 8) | (buffer[i + 3]);
        memcpy(&(buffer[i]), &tmp, 4);
    }
}

uint8_t WaitMs(VL53L5CX_Platform* platform, uint32_t time)
{
    sleep_ms(time);
    return VL53L5CX_OK;
}
