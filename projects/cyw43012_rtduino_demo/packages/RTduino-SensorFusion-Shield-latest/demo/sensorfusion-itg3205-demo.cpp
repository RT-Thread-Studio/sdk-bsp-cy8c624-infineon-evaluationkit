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
#include "ITG3200.h"

ITG3200 gyro;

static void _itg3205_setup(void)
{
    /* put your setup code here, to run once: */
    Serial.begin(115200);
    gyro.init();
    gyro.zeroCalibrate(200, 10); //sample 200 times to calibrate and it will take 200*10ms
}

static void _itg3205_loop(void)
{
    /* put your main code here, to run repeatedly: */
    int x, y, z;
    gyro.getXYZ(&x, &y, &z);
    Serial.print("values of X:");
    Serial.print(x);
    Serial.print(", Y:");
    Serial.print(y);
    Serial.print(", Z");
    Serial.println(z);

    float ax, ay, az;
    gyro.getAngularVelocity(&ax, &ay, &az);
    Serial.print("Angular Velocity of X:");
    Serial.print(ax);
    Serial.print(", Y:");
    Serial.print(ay);
    Serial.print(", Z:");
    Serial.print(az);
    Serial.println(" degrees per second");
    Serial.println("*************");
    delay(1000);
}
RTDUINO_SKETCH_LOADER("itg3205", _itg3205_setup, _itg3205_loop);