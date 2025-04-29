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
#include "Adafruit_SHT31.h"

static float sht31_t;
static float sht31_h;
static bool enableHeater = false;
static uint8_t loopCnt = 0;

static Adafruit_SHT31 sht31 = Adafruit_SHT31();

static void _sht31_setup(void)
{
    /* put your setup code here, to run once: */
    Serial.println("SHT31 test");
    if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
        Serial.println("Couldn't find SHT31");
        while (1) delay(1);
    }

    Serial.print("Heater Enabled State: ");
    if (sht31.isHeaterEnabled())
        Serial.println("ENABLED");
    else
        Serial.println("DISABLED");
}

static void _sht31_loop(void)
{
    /* put your main code here, to run repeatedly: */
    sht31_t = sht31.readTemperature();
    sht31_h = sht31.readHumidity();

    if (! isnan(sht31_t)) {  // check if 'is not a number'
        Serial.print("SHT31:   Temp =   "); Serial.print(sht31_t); Serial.print("\t");
    } else { 
        Serial.println("Failed to read temperature");
    }
    
    if (! isnan(sht31_h)) {  // check if 'is not a number'
        Serial.print("Hum  = "); Serial.println(sht31_h);
        Serial.println();
    } else { 
        Serial.println("Failed to read humidity");
    }

    delay(1000);

    // Toggle heater enabled state every 30 seconds
    // An ~3.0 degC temperature increase can be noted when heater is enabled
    if (loopCnt >= 30) {
        enableHeater = !enableHeater;
        sht31.heater(enableHeater);
        Serial.print("Heater Enabled State: ");
        if (sht31.isHeaterEnabled())
        Serial.println("ENABLED");
        else
        Serial.println("DISABLED");

        loopCnt = 0;
    }
    loopCnt++;
}
RTDUINO_SKETCH_LOADER("sht31", _sht31_setup, _sht31_loop);
