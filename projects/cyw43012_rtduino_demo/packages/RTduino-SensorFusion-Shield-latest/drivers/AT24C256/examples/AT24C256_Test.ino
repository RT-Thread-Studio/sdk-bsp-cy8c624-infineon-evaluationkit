#include <AT24C256.h>

// 初始化 EEPROM 对象，I2C 地址为 0x50
AT24C256 eeprom(0x50);

void setup() {
    Serial.begin(9600);
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

void loop() {
    // 不需要在 loop 中做任何操作
}
