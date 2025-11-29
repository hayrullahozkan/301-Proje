#include "vl53l0x.h"
#include <Wire.h>

VL53L0XDriver::VL53L0XDriver(uint8_t xshutPin)
{
    _xshut = xshutPin;
}

bool VL53L0XDriver::begin()
{
    // XSHUT = sensörü aç
    pinMode(_xshut, OUTPUT);
    digitalWrite(_xshut, HIGH);
    delay(10);

    // I2C başlat
    Wire.begin();   // Mega: SCL=21, SDA=20 otomatik

    if (!sensor.init()) {
        return false;
    }

    sensor.setTimeout(500);
    sensor.startContinuous(50);   // 50ms per reading
    return true;
}

int VL53L0XDriver::readContinuous()
{
    return sensor.readRangeContinuousMillimeters();
}
