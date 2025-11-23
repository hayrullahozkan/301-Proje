#include "vl53l0x.h"
#include <Wire.h>

// Registers (minimal set)
#define SYSRANGE_START 0x00
#define RESULT_RANGE_STATUS 0x14

VL53L0XDriver::VL53L0XDriver(uint8_t xshutPin)
{
    _xshutPin = xshutPin;
    _last_mm  = -1;
}

void VL53L0XDriver::begin()
{
    Wire.begin();

    // Handle XSHUT if provided
    if (_xshutPin != 255) {
        pinMode(_xshutPin, OUTPUT);
        digitalWrite(_xshutPin, LOW);
        delay(5);
        digitalWrite(_xshutPin, HIGH);
        delay(5);
    }

    // Basic init
    writeReg(0x88, readReg(0x88) | 0x01);

    // Start continuous mode
    startContinuous(20);
}

void VL53L0XDriver::shutdown()
{
    if (_xshutPin != 255)
        digitalWrite(_xshutPin, LOW);
}

void VL53L0XDriver::wakeUp()
{
    if (_xshutPin != 255)
        digitalWrite(_xshutPin, HIGH);
}

void VL53L0XDriver::startContinuous(uint16_t period_ms)
{
    writeReg(0x01, 0x02);      // Start continuous mode
    writeReg(0x04, (period_ms >> 8) & 0xFF);
    writeReg(0x05,  period_ms & 0xFF);
}

int VL53L0XDriver::readContinuous()
{
    Wire.beginTransmission(VL53L0X_I2C_ADDR);
    Wire.write(RESULT_RANGE_STATUS + 10);
    Wire.endTransmission();

    Wire.requestFrom(VL53L0X_I2C_ADDR, 2);
    if (Wire.available() < 2) return _last_mm;

    uint16_t mm = Wire.read() << 8;
    mm |= Wire.read();

    _last_mm = (int)mm;
    return _last_mm;
}

int VL53L0XDriver::lastReading()
{
    return _last_mm;
}

void VL53L0XDriver::writeReg(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(VL53L0X_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t VL53L0XDriver::readReg(uint8_t reg)
{
    Wire.beginTransmission(VL53L0X_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(VL53L0X_I2C_ADDR, 1);
    if (!Wire.available()) return 0;
    return Wire.read();
}
