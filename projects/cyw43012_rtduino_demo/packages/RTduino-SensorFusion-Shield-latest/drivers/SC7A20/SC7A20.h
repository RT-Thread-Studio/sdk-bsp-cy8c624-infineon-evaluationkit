#ifndef SC7A20_H
#define SC7A20_H

#include <Wire.h>

#define SC7A20_ADDRESS 0x18

// Register Addresses
#define SC7A20_REG_WHO_AM_I 0x0F
#define SC7A20_REG_CTRL_REG1 0x20
#define SC7A20_REG_CTRL_REG2 0x21
#define SC7A20_REG_CTRL_REG3 0x22
#define SC7A20_REG_CTRL_REG4 0x23
#define SC7A20_REG_CTRL_REG5 0x24
#define SC7A20_REG_OUT_X_L 0x28
#define SC7A20_REG_INT1_CFG 0x30
#define SC7A20_REG_INT1_SRC 0x31
#define SC7A20_REG_INT1_THS 0x32
#define SC7A20_REG_INT1_DURATION 0x33
#define SC7A20_REG_FIFO_CTRL 0x2E
#define SC7A20_REG_FIFO_SRC 0x2F

class SC7A20 {
public:
    SC7A20();
    bool begin();
    
    // Basic Functions
    void readAcceleration(int16_t &x, int16_t &y, int16_t &z);
    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t value);

    // Configuration Functions
    void setDataRate(uint8_t rate);
    void setFullScale(uint8_t scale);
    void enableHighPassFilter(bool enable);
    void setInterruptThreshold(uint8_t threshold);
    void setInterruptDuration(uint8_t duration);
    void configureFIFO(uint8_t mode, uint8_t threshold);

private:
    void initSensor();
};

#endif
