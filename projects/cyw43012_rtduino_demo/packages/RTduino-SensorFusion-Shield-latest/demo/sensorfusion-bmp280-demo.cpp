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
#include <SPI.h>
#include <Adafruit_BMP280.h>

static float bmp280_temperature, bmp280_pressure;
#define BMP280_I2C_ADDRESS 0x76  // BMP280的I2C地址
// BMP280寄存器地址
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_CALIB 0x88  // 校准数据起始地址

// 校准参数
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

float compensateTemperature(int32_t adc_T) {
  int32_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  int32_t t_fine = var1 + var2;
  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

float compensatePressure(int32_t adc_P, int32_t t_fine) {
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

  if (var1 == 0) {
    return 0;  // 避免除以零
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (float)p / 256;
}

uint16_t read16_LE() {
  uint16_t value = Wire.read();
  return (Wire.read() << 8) | value;
}

int16_t readS16_LE() {
  return (int16_t)read16_LE();
}

void readCalibrationData() {
  Wire.beginTransmission(BMP280_I2C_ADDRESS);
  Wire.write(BMP280_REG_CALIB);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_I2C_ADDRESS, 24);

  dig_T1 = read16_LE();
  dig_T2 = readS16_LE();
  dig_T3 = readS16_LE();
  dig_P1 = read16_LE();
  dig_P2 = readS16_LE();
  dig_P3 = readS16_LE();
  dig_P4 = readS16_LE();
  dig_P5 = readS16_LE();
  dig_P6 = readS16_LE();
  dig_P7 = readS16_LE();
  dig_P8 = readS16_LE();
  dig_P9 = readS16_LE();
}

bool readBMP280(float &temperature, float &pressure) {
  uint8_t data[6];

  Wire.beginTransmission(BMP280_I2C_ADDRESS);
  Wire.write(BMP280_REG_PRESS_MSB);
  Wire.endTransmission();

  Wire.requestFrom(BMP280_I2C_ADDRESS, 6);
  if (Wire.available() == 6) {
    for (int i = 0; i < 6; i++) {
      data[i] = Wire.read();
    }

    int32_t adc_P = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4));

    temperature = compensateTemperature(adc_T);
    pressure = compensatePressure(adc_P, temperature);

    return true;
  } else {
    return false;
  }
}

static void _bmp280_setup(void)
{
    /* put your setup code here, to run once: */
    Wire.begin();  // 初始化I2C通信

    // 读取校准数据
    readCalibrationData();

    // 初始化BMP280
    Wire.beginTransmission(BMP280_I2C_ADDRESS);
    Wire.write(BMP280_REG_CTRL_MEAS);
    Wire.write(0x27);  // 设置温度和压力的过采样率和模式
    Wire.endTransmission();

    Wire.beginTransmission(BMP280_I2C_ADDRESS);
    Wire.write(BMP280_REG_CONFIG);
    Wire.write(0xA0);  // 设置滤波器系数和待机时间
    Wire.endTransmission();
}

static void _bmp280_loop(void)
{
    /* put your main code here, to run repeatedly: */
    if (readBMP280(bmp280_temperature, bmp280_pressure)) {
        Serial.print("BMP280:  Temperature: ");
        Serial.print(bmp280_temperature);
        Serial.print(" *C  ");

        Serial.print("Pressure: ");
        Serial.print(bmp280_pressure);
        Serial.println(" hPa");
        Serial.println();
    } else {
        Serial.println("Failed to read data from BMP280.");
    }

    delay(1000);  // 每秒读取一次

}
RTDUINO_SKETCH_LOADER("bmp280", _bmp280_setup, _bmp280_loop);
