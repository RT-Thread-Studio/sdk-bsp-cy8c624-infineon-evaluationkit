#include <Wire.h>
#include "SC7A20.h"

static SC7A20 accel;

void setup() 
{
    Serial.begin(115200);
    if (!accel.begin()) {
        Serial.println("SC7A20 not detected!");
        while (1);
    }
    Serial.println("SC7A20 detected!");

    // 配置示例
    accel.setDataRate(0x50);  // 设置为 50Hz
    accel.setFullScale(0x10); // 设置为 ±4g
    accel.enableHighPassFilter(true); // 启用高通滤波器
    accel.setInterruptThreshold(0x10); // 设置中断阈值
    accel.setInterruptDuration(0x01);  // 设置中断持续时间
    accel.configureFIFO(0x02, 0x1F);   // FIFO 流模式，阈值31
}

void loop() 
{  
    int16_t x, y, z;
    accel.readAcceleration(x, y, z);
    
    Serial.print("X: "); Serial.print(x);
    Serial.print(" Y: "); Serial.print(y);
    Serial.print(" Z: "); Serial.println(z);
    
    delay(500);
}