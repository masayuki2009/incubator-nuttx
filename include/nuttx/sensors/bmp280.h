/****************************************************************************
 * include/nuttx/sensors/bmp280.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_BMP280_H
#define __INCLUDE_NUTTX_SENSORS_BMP280_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && (defined(CONFIG_SENSORS_BMP280) || defined(CONFIG_SENSORS_BMP280_SCU))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *
 * CONFIG_BMP280
 *   Enables support for the BMP280 driver
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* IOCTL Commands ***********************************************************/

/* Arg: 0: Disable compensated
 *      1: Enable compensated
 */

#define ENABLE_COMPENSATED (1)
#define DISABLE_COMPENSATED (0)

/* Standby duration */

#define BMP280_STANDBY_05_MS   (0x00) /* 0.5 ms */
#define BMP280_STANDBY_63_MS   (0x01) /* 62.5 ms */
#define BMP280_STANDBY_125_MS  (0x02) /* 125 ms */
#define BMP280_STANDBY_250_MS  (0x03) /* 250 ms */
#define BMP280_STANDBY_500_MS  (0x04) /* 500 ms */
#define BMP280_STANDBY_1000_MS (0x05) /* 1000 ms */
#define BMP280_STANDBY_2000_MS (0x06) /* 2000 ms */
#define BMP280_STANDBY_4000_MS (0x07) /* 4000 ms */

/* Enable compensate of sensing values (no SCU bus only)
 *
 * Arg: ENABLE_COMPENSATED or DISABLE_COMPENSATED
 */

#define SNIOC_ENABLE_COMPENSATED   _SNIOC(0x0001)

/* Get sensor predefined adjustment values (SCU bus only)
 *
 * Arg: Pointer of struct bmp280_press_adj_s (pressure)
 *      Pointer of struct bmp280_temp_adj_s (temperature)
 */

#define SNIOC_GETADJ               _SNIOC(0x0002)

/* Set sensor standby duration
 *
 * Arg: BMP280_STANDBY_*_MS
 */

#define SNIOC_SETSTB               _SNIOC(0x0003)

/* Get temperature value
 *
 * Arg: Pointer to uint32_t (raw value)
 */

#define SNIOC_GET_TEMP             _SNIOC(0x0004)

struct bmp280_press_adj_s
{
  uint16_t  dig_p1; /* calibration P1 data */
  int16_t   dig_p2; /* calibration P2 data */
  int16_t   dig_p3; /* calibration P3 data */
  int16_t   dig_p4; /* calibration P4 data */
  int16_t   dig_p5; /* calibration P5 data */
  int16_t   dig_p6; /* calibration P6 data */
  int16_t   dig_p7; /* calibration P7 data */
  int16_t   dig_p8; /* calibration P8 data */
  int16_t   dig_p9; /* calibration P9 data */
};

struct bmp280_temp_adj_s
{
  uint16_t  dig_t1; /* calibration T1 data */
  int16_t   dig_t2; /* calibration T2 data */
  int16_t   dig_t3; /* calibration T3 data */
};

struct bmp280_meas_s
{
  uint8_t   msb;    /* meas value MSB */
  uint8_t   lsb;    /* meas value LSB */
  uint8_t   xlsb;   /* meas value XLSB */
};

/****************************************************************************
 * Name: bmp280_register
 *
 * Description:
 *   Register the BMP280 character device
 *
 * Input Parameters:
 *   devno   - Instance number for driver
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BMP280
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp280_register(int devno, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP280 */
#endif /* __INCLUDE_NUTTX_BMP280_H */
