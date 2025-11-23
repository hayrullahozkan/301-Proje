#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float Ts);

    void setGains(float Kp, float Ki, float Kd);
    void getGains(float &Kp, float &Ki, float &Kd);

    float compute(float ref, float meas, int id);

    void reset();   // integral ve error reset

private:
    float _Ts;
    float _Kp;
    float _Ki;
    float _Kd;

    float _integral[4];
    float _prevError[4];
};

#endif
