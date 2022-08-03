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
 * @file ESP32_BH1750.h
 *
 * @defgroup BH1750 BH1750
 * @{
 *
 * ESP-IDF driver for ESP32_BH1750 light sensor
 *
 * Datasheet: ROHM Semiconductor bh1750fvi-e.pdf
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2017 Andrej Krutak <dev@andree.sk>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>\n
 * Copyright (c) 2022 Simone Tollardo <simonetollardo@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __ESP32_BH1750_H__
#define __ESP32_BH1750_H__

#include <stdint.h>
#include "driver/i2c.h"
#include <esp_err.h>

#define BH1750_ADDR_LO (0x23) //!< I2C address when ADDR pin floating/low
#define BH1750_ADDR_HI (0x5c) //!< I2C address when ADDR pin high

extern uint8_t BH1750_SDA_PIN;
extern uint8_t BH1750_SCL_PIN;
extern uint8_t BH1750_I2C_PORT;

/**
 * Measurement mode
 */
typedef enum
{
    BH1750_MODE_ONE_TIME = 0, //!< One time measurement
    BH1750_MODE_CONTINUOUS    //!< Continuous measurement
} bh1750_mode_t;

/**
 * Measurement resolution
 */
typedef enum
{
    BH1750_RES_LOW = 0,  //!< 4 lx resolution, measurement time is usually 16 ms
    BH1750_RES_HIGH,     //!< 1 lx resolution, measurement time is usually 120 ms
    BH1750_RES_HIGH2     //!< 0.5 lx resolution, measurement time is usually 120 ms
} bh1750_resolution_t;

typedef struct{
  i2c_config_t conf;
  bool _tcs34725Initialised;
  bh1750_mode_t mode;
  bh1750_resolution_t res;
  uint8_t addr;
  uint8_t MTime;
  uint8_t data[2];
} ESP32_BH1750;

esp_err_t BH1750_init(ESP32_BH1750 *ESP32_BH1750, uint8_t addr);
esp_err_t BH1750_delete();
void BH1750_power_down(ESP32_BH1750 *ESP32_BH1750);
void BH1750_power_on(ESP32_BH1750 *ESP32_BH1750);
void BH1750_reset(ESP32_BH1750 *ESP32_BH1750);
void BH1750_set(ESP32_BH1750 *ESP32_BH1750, bh1750_mode_t mode, bh1750_resolution_t res, uint8_t time);
void BH1750_measure(ESP32_BH1750 *ESP32_BH1750, bh1750_mode_t mode, bh1750_resolution_t res, uint8_t time);
void BH1750_read_measure(ESP32_BH1750 *ESP32_BH1750, uint16_t *level);
void BH1750_measure_and_read(ESP32_BH1750 *ESP32_BH1750, uint16_t *level);

#endif /* __BH1750_H__ */
