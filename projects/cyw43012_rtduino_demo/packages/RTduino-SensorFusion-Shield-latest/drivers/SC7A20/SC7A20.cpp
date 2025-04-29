#include "SC7A20.h"

SC7A20::SC7A20() {}

bool SC7A20::begin() {
    Wire.begin();
    if (readRegister8(SC7A20_REG_WHO_AM_I) == 0x11) {  // WHO_AM_I should return 0x11
        initSensor();
        return true;
    }
    return false;
}

void SC7A20::initSensor() {
    setDataRate(0x57);  // Default ODR = 100Hz, all axes enabled
    setFullScale(0x00); // Default ±2g full scale
}

void SC7A20::readAcceleration(int16_t &x, int16_t &y, int16_t &z) {
    uint8_t xl = readRegister8(SC7A20_REG_OUT_X_L);
    uint8_t xh = readRegister8(SC7A20_REG_OUT_X_L + 1);
    x = ((int16_t)xh << 8) | xl;

    uint8_t yl = readRegister8(SC7A20_REG_OUT_X_L + 2);
    uint8_t yh = readRegister8(SC7A20_REG_OUT_X_L + 3);
    y = ((int16_t)yh << 8) | yl;

    uint8_t zl = readRegister8(SC7A20_REG_OUT_X_L + 4);
    uint8_t zh = readRegister8(SC7A20_REG_OUT_X_L + 5);
    z = ((int16_t)zh << 8) | zl;
}

uint8_t SC7A20::readRegister8(uint8_t reg) {
    Wire.beginTransmission(SC7A20_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(SC7A20_ADDRESS, 1);
    return Wire.read();
}

void SC7A20::writeRegister8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(SC7A20_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void SC7A20::setDataRate(uint8_t rate) {
    uint8_t reg = readRegister8(SC7A20_REG_CTRL_REG1);
    reg &= 0x0F;  // Clear previous data rate bits
    reg |= (rate & 0xF0); // Set new data rate
    writeRegister8(SC7A20_REG_CTRL_REG1, reg);
}

void SC7A20::setFullScale(uint8_t scale) {
    uint8_t reg = readRegister8(SC7A20_REG_CTRL_REG4);
    reg &= 0xCF;  // Clear previous scale bits
    reg |= (scale & 0x30); // Set new scale
    writeRegister8(SC7A20_REG_CTRL_REG4, reg);
}

void SC7A20::enableHighPassFilter(bool enable) {
    uint8_t reg = readRegister8(SC7A20_REG_CTRL_REG2);
    if (enable) {
        reg |= 0x10;  // Enable high-pass filter
    } else {
        reg &= ~0x10; // Disable high-pass filter
    }
    writeRegister8(SC7A20_REG_CTRL_REG2, reg);
}

void SC7A20::setInterruptThreshold(uint8_t threshold) {
    writeRegister8(SC7A20_REG_INT1_THS, threshold);
}

void SC7A20::setInterruptDuration(uint8_t duration) {
    writeRegister8(SC7A20_REG_INT1_DURATION, duration);
}

void SC7A20::configureFIFO(uint8_t mode, uint8_t threshold) {
    writeRegister8(SC7A20_REG_FIFO_CTRL, (mode << 6) | (threshold & 0x1F));
}
