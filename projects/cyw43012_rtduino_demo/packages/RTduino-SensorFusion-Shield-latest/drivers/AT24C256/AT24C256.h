#ifndef AT24C256_H
#define AT24C256_H

#include <Arduino.h>
#include <Wire.h>

class AT24C256 {
public:
    AT24C256(uint8_t deviceAddress);

    void begin();
    void writeByte(uint16_t address, uint8_t data);
    uint8_t readByte(uint16_t address);

    void writePage(uint16_t address, const uint8_t* data, uint16_t length);
    void readPage(uint16_t address, uint8_t* buffer, uint16_t length);

    void writeString(uint16_t address, const char* data);
    void readString(uint16_t address, char* buffer, uint16_t length);

    void clearMemory(uint8_t value = 0xFF);
    uint16_t length();

private:
    uint8_t _deviceAddress;
    uint16_t _maxAddress;

    void waitForWriteComplete();
};

#endif
