#include "AT24C256.h"

AT24C256::AT24C256(uint8_t deviceAddress)
    : _deviceAddress(deviceAddress), _maxAddress(32767) {}

void AT24C256::begin() {
    Wire.begin();
}

void AT24C256::writeByte(uint16_t address, uint8_t data) {
    Wire.beginTransmission(_deviceAddress);
    Wire.write((int)(address >> 8));   // 高字节地址
    Wire.write((int)(address & 0xFF)); // 低字节地址
    Wire.write(data);
    Wire.endTransmission();
    waitForWriteComplete();
}

uint8_t AT24C256::readByte(uint16_t address) {
    Wire.beginTransmission(_deviceAddress);
    Wire.write((int)(address >> 8));   // 高字节地址
    Wire.write((int)(address & 0xFF)); // 低字节地址
    Wire.endTransmission();
    Wire.requestFrom(_deviceAddress, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF; // 如果读取失败，返回 0xFF
}

void AT24C256::writePage(uint16_t address, const uint8_t* data, uint16_t length) {
    uint16_t pageBoundary = (address / 64 + 1) * 64;
    uint16_t chunkSize = min(length, pageBoundary - address);

    Wire.beginTransmission(_deviceAddress);
    Wire.write((int)(address >> 8));   // 高字节地址
    Wire.write((int)(address & 0xFF)); // 低字节地址
    for (uint16_t i = 0; i < chunkSize; i++) {
        Wire.write(data[i]);
    }
    Wire.endTransmission();
    waitForWriteComplete();

    if (length > chunkSize) {
        writePage(pageBoundary, data + chunkSize, length - chunkSize);
    }
}

void AT24C256::readPage(uint16_t address, uint8_t* buffer, uint16_t length) {
    uint16_t pageBoundary = (address / 64 + 1) * 64;
    uint16_t chunkSize = min(length, pageBoundary - address);

    Wire.beginTransmission(_deviceAddress);
    Wire.write((int)(address >> 8));   // 高字节地址
    Wire.write((int)(address & 0xFF)); // 低字节地址
    Wire.endTransmission();
    waitForWriteComplete(); // 等待操作完成，增加稳定性

    Wire.requestFrom(_deviceAddress, chunkSize);
    for (uint16_t i = 0; i < chunkSize; i++) {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        } else {
            buffer[i] = 0xFF; // 如果读取失败，填充 0xFF
        }
    }

    if (length > chunkSize) {
        readPage(pageBoundary, buffer + chunkSize, length - chunkSize);
    }
}

void AT24C256::writeString(uint16_t address, const char* data) {
    while (*data) {
        writeByte(address++, *data++);
    }
    writeByte(address, 0); // 空终止符
}

void AT24C256::readString(uint16_t address, char* buffer, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = readByte(address++);
        if (buffer[i] == 0) break; // 碰到空终止符时停止读取
    }
}

void AT24C256::clearMemory(uint8_t value) {
    for (uint16_t i = 0; i <= _maxAddress; i++) {
        writeByte(i, value);
    }
}

uint16_t AT24C256::length() {
    return _maxAddress + 1;
}

void AT24C256::waitForWriteComplete() {
    delay(10); // 增加延迟，确保写入周期完成
}
