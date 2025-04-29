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
#include <Adafruit_ADXL345_U.h>

sensors_event_t adxl245_event; 
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void displayRange(void);
void displayDataRate(void);
void displaySensorDetails(void);

static void _adxl345_setup(void)
{
    /* put your setup code here, to run once: */
    Serial.println("Accelerometer Test"); Serial.println("");
  
    /* Initialise the sensor */
    if(!accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while(1);
    }
    accel.setRange(ADXL345_RANGE_16_G);
    displaySensorDetails();
    displayDataRate();
    displayRange();
    Serial.println("");
}

static void _adxl345_loop(void)
{
    /* put your main code here, to run repeatedly: */
    accel.getEvent(&adxl245_event);
    
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("ADXL345:  ");
    Serial.print("X: "); Serial.print(adxl245_event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(adxl245_event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(adxl245_event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
    Serial.println();
    delay(1000);
}
RTDUINO_SKETCH_LOADER("bmp280", _adxl345_setup, _adxl345_loop);

void displayRange(void)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? ");
      break;
  }  
  Serial.println(" Hz");  
}