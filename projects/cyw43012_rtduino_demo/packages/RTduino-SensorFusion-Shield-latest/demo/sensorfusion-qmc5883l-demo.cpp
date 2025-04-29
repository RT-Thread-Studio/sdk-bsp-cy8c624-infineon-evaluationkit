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
#include <MechaQMC5883.h>

static MechaQMC5883 qmc;

static void _qmc5883l_setup(void)
{
    /* put your setup code here, to run once: */
    Wire.begin();
    Serial.begin(115200);
    qmc.init();
    //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
}

static void _qmc5883l_loop(void)
{
  // Retrive the raw values from the compass (not scaled).
  int x, y, z;
  int azimuth;
  //float azimuth; //is supporting float too
  qmc.read(&x, &y, &z,&azimuth);
  //azimuth = qmc.azimuth(&y,&x);//you can get custom azimuth
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" z: ");
  Serial.print(z);
  Serial.print(" a: ");
  Serial.print(azimuth);
  Serial.println();
  delay(100);
}
RTDUINO_SKETCH_LOADER("qmc5883l", _qmc5883l_setup, _qmc5883l_loop);
