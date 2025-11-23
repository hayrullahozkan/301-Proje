#ifndef VL53L0X_DRIVER_H
#define VL53L0X_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

// ------------------------------
// VL53L0X Default I2C Address
// Datasheet: Address = 0x52  (7-bit I2C = 0x29)
// ------------------------------
#define VL53L0X_I2C_ADDR 0x29   // 7-bit address

// ------------------------------
// Pin configuration
// ------------------------------
class VL53L0XDriver {
public:
    VL53L0XDriver(uint8_t xshutPin = 255);

    void begin();
    void shutdown();
    void wakeUp();
    void startContinuous(uint16_t period_ms = 20);
    int  readContinuous();        // returns mm
    int  lastReading();           // cached reading

private:
    uint8_t _xshutPin;
    int     _last_mm;

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
};

#endif
