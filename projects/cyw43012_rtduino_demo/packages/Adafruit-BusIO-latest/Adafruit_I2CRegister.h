/*
 * Copyright (c) 2021-2022, RTduino Development Team
 *
 * SPDX-License-Identifier: MIT
 *
 * https://github.com/RTduino/RTduino
 * https://gitee.com/rtduino/RTduino
 *
 * Change Logs:
 * Date           Author       Notes
 *            Adafruit Team    First version
 * 2022-02-19     Meco Man     port to RTduino
 */

#ifndef _ADAFRUIT_I2C_REGISTER_H_
#define _ADAFRUIT_I2C_REGISTER_H_

#include <Arduino.h>

#ifdef RTDUINO_USING_WIRE
#include <Adafruit_BusIO_Register.h>

typedef Adafruit_BusIO_Register Adafruit_I2CRegister;
typedef Adafruit_BusIO_RegisterBits Adafruit_I2CRegisterBits;

#endif /* RTDUINO_USING_WIRE */
#endif /* _ADAFRUIT_I2C_REGISTER_H_ */
