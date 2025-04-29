/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-08-18     Wangyuqiang  first version
 */

#include <RTduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>

static sensors_event_t aht20_humidity, aht20_temp;
static Adafruit_AHTX0 aht;

static void _aht20_setup(void)
{
    /* put your setup code here, to run once: */
    if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
    }
    Serial.println("AHT10 or AHT20 found");
}

static void _aht20_loop(void)
{
    /* put your main code here, to run repeatedly: */
    aht.getEvent(&aht20_humidity, &aht20_temp);// populate temp and humidity objects with fresh data
    Serial.print("AHT20:  Temperature: "); Serial.print(aht20_temp.temperature); Serial.print(" degrees C        ");
    Serial.print("Humidity: "); Serial.print(aht20_humidity.relative_humidity); Serial.println("% rH");
    Serial.println();
    delay(500);
}
RTDUINO_SKETCH_LOADER("aht20", _aht20_setup, _aht20_loop);
