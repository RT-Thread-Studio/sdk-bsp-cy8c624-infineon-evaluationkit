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
#include "AT24C256.h"

// 初始化 EEPROM 对象，I2C 地址为 0x50
AT24C256 eeprom(0x50);

static void _at24c256_setup(void)
{
    /* put your setup code here, to run once: */
    Serial.begin(115200);
    eeprom.begin();

    // 写入并读取字符串
    const char* message = "Hello, EEPROM!";
    eeprom.writeString(0, message);

    char buffer[20];
    eeprom.readString(0, buffer, sizeof(buffer));
    Serial.println(buffer);

    // 测试页面写入和读取
    uint8_t pageData[32];
    for (uint8_t i = 0; i < 32; i++) {
        pageData[i] = i;
    }
    eeprom.writePage(100, pageData, 32);

    uint8_t readPageData[32];
    eeprom.readPage(100, readPageData, 32);

    for (uint8_t i = 0; i < 32; i++) {
        Serial.print(readPageData[i]);
        Serial.print(" ");
    }
    Serial.println();

    // 擦除 EEPROM
    eeprom.clearMemory();
}

static void _at24c256_loop(void)
{
    /* put your main code here, to run repeatedly: */
    delay(50);
}
RTDUINO_SKETCH_LOADER("at24c256", _at24c256_setup, _at24c256_loop);