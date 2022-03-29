/*
 * Copyright (c) 2017 Andrej Krutak <dev@andree.sk>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2022 Simone Tollardo <simonetollardo@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file bh1750.c
 *
 * @ingroup bh1750 ESP-IDF driver for BH1750 light sensor
 *
 * ESP-IDF driver for BH1750 light sensor
 *
 * Datasheet: ROHM Semiconductor bh1750fvi-e.pdf
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2017 Andrej Krutak <dev@andree.sk>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>\n
 * Copyright (c) 2022 Simone Tollardo <simonetollardo@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include "bh1750.h"


#define OPCODE_HIGH  (0x0)
#define OPCODE_HIGH2 (0x1)
#define OPCODE_LOW   (0x3)

#define OPCODE_CONT (0x10)
#define OPCODE_OT   (0x20)

#define OPCODE_RESET (0x07)

#define OPCODE_POWER_DOWN (0x00)
#define OPCODE_POWER_ON   (0x01)
#define OPCODE_MT_HI      (0x40)    //Measurement time High Bits -> see datasheet
#define OPCODE_MT_LO      (0x60)    //Measurement time Low Bits -> see datasheet
#define MTIME_DEFAULT     (0x45)    //Default measurement time

#define I2C_FREQ_HZ (400000)

static const char* TAG="BH1750";

//Support functions

/**
 *  @brief  Writes a register and an 8 bit value over I2C
 *  @param  reg
 *  @param  value
 */
void write8(BH1750* BH1750, uint8_t opcode) {
    uint8_t buffer[1];
    buffer[0]=opcode;
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, BH1750->addr, buffer, 1, 1000 / portTICK_RATE_MS));
}

//Public functions

/**
 * @brief Initialize device descriptor
 *
 * @param[out] ESP_BH1750 BH1750 Structure
 * @param[in] addr I2C address, BH1750_ADDR_LO or BH1750_ADDR_HI
 * @param[in] port I2C port number
 * @param[in] sda_gpio GPIO pin number for SDA
 * @param[in] scl_gpio GPIO pin number for SCL
 * @return `ESP_OK` on success
 */
esp_err_t bh1750_init_desc(BH1750 *BH1750, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{   
    if (addr != BH1750_ADDR_LO && addr != BH1750_ADDR_HI)
    {
        ESP_LOGE(TAG,"Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }
    BH1750->addr = addr;
    BH1750->_tcs34725Initialised = true;
    BH1750->mode = BH1750_MODE_CONTINUOUS;
    BH1750->res = BH1750_RES_HIGH;
    BH1750->MTime = MTIME_DEFAULT;

    BH1750->conf.mode = I2C_MODE_MASTER;
    BH1750->conf.sda_io_num = SDA_PIN;
    BH1750->conf.scl_io_num = SCL_PIN;
    BH1750->conf.sda_pullup_en = GPIO_PULLUP_DISABLE;     //disable if you have external pullup
    BH1750->conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    BH1750->conf.master.clk_speed = 400000;               //I2C Full Speed

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &(BH1750->conf))); //set I2C Config

    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
    
    bh1750_set_and_measure(BH1750,BH1750->mode,BH1750->res);
    bh1750_set_measurement_time(BH1750,BH1750->MTime);
    
    return ESP_OK;
}

/**
 * @brief Set measurement time
 *
 * @param ESP_BH1750 BH1750 Structure
 * @param time Measurement time (see datasheet)
 * 
 */
void bh1750_set_measurement_time(BH1750 *BH1750, uint8_t time)
{   
    BH1750->MTime = time;
    uint8_t MTHigh = OPCODE_MT_HI | (time >> 5);
    uint8_t MTLow = OPCODE_MT_LO | (time & 0x1F);
    write8(BH1750, MTHigh);
    write8(BH1750, MTLow);
}

/**
 * @brief Power down device
 *
 * @param ESP_BH1750 BH1750 Structure
 * 
 */
void bh1750_power_down(BH1750 *BH1750)
{
    write8(BH1750, OPCODE_POWER_DOWN);
}

/**
 * @brief Power on device
 *
 * @param ESP_BH1750 BH1750 Structure
 * 
 */
void bh1750_power_on(BH1750 *BH1750)
{
    write8(BH1750, OPCODE_POWER_ON);
}


/**
 * @brief Reset bh1750. This function cannot be called when device is powered off.
 *
 * @param ESP_BH1750 BH1750 Structure
 * 
 */
void bh1750_reset(BH1750 *BH1750)
{
    write8(BH1750, OPCODE_RESET);
}

/**
 * @brief Setup device parameters
 *
 * @param ESP_BH1750 BH1750 Structure
 * @param mode Measurement mode
 * @param resolution Measurement resolution
 * 
 */
void bh1750_set_and_measure(BH1750 *BH1750, bh1750_mode_t mode, bh1750_resolution_t resolution)
{
    uint8_t opcode = mode == BH1750_MODE_CONTINUOUS ? OPCODE_CONT : OPCODE_OT;
    switch (resolution)
    {
        case BH1750_RES_LOW:  opcode |= OPCODE_LOW;   break;
        case BH1750_RES_HIGH: opcode |= OPCODE_HIGH;  break;
        default:              opcode |= OPCODE_HIGH2; break;
    }

    write8(BH1750, opcode);
}

/**
 * @brief Read measured value. Note that this function should be called after bh1750_set_and_measure().
 * You must wait the measurement time before calling this function or instead use bh1750_measure_and_read().
 * @param ESP_BH1750 BH1750 Structure
 * @param level Uint16_t pointer to store measured value
 * 
 */
void bh1750_read_measure(BH1750 *BH1750, uint16_t *level){
    uint8_t buffer[2];
    ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_PORT, BH1750->addr, buffer, 2, 1000 / portTICK_RATE_MS));

    *level = buffer[0] << 8 | buffer[1];
    *level = (*level * 10) / 12; // convert to LUX
}

/**
 *  @brief  Complete a measure (wait the necessary time) and read the value.
 *  If the BH1750 is set in OneTime measurement mode, you need to do a 
 *  bh1750_power_on() before calling thus function.
 *  @param level Uint16_t pointer to store measured value
 */
void bh1750_measure_and_read(BH1750* BH1750, uint16_t *level) {
    uint8_t buffer[2];
    uint8_t wait_time_ms=0;

    //TODO:Testare se effettivamente servono le "set_and_measure"
    if(BH1750->mode == BH1750_MODE_ONE_TIME) {
        bh1750_power_on(BH1750);
        bh1750_set_and_measure(BH1750,BH1750->mode,BH1750->res);
    }
    else if(BH1750->mode == BH1750_MODE_CONTINUOUS) {
        bh1750_set_and_measure(BH1750,BH1750->mode,BH1750->res);
    }

    switch (BH1750->res)
    {
    case BH1750_RES_LOW:  
        wait_time_ms = 24*(BH1750->MTime/((uint8_t)MTIME_DEFAULT));
        break;
    case BH1750_RES_HIGH:
        wait_time_ms = 180*(BH1750->MTime/((uint8_t)MTIME_DEFAULT));
        break;
    case BH1750_RES_HIGH2:
        wait_time_ms = 180*(BH1750->MTime/((uint8_t)MTIME_DEFAULT));
        break;
    default:
        wait_time_ms=0;
        ESP_LOGE(TAG, "bh1750_measure_and_read: Invalid resolution");
        break;
    }
    vTaskDelay(wait_time_ms/portTICK_RATE_MS);
    ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_PORT, BH1750->addr, buffer, 2, 1000 / portTICK_RATE_MS));

    *level = buffer[0] << 8 | buffer[1];
    *level = (*level * 10) / 12; // convert to LUX

}