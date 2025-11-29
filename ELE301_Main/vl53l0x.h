#ifndef VL53L0X_DRIVER_H
#define VL53L0X_DRIVER_H

#include <Arduino.h>
#include <VL53L0X.h>

class VL53L0XDriver {
public:
    VL53L0XDriver(uint8_t xshutPin);

    bool begin();
    int readContinuous();

private:
    uint8_t _xshut;
    VL53L0X sensor;
};

#endif
