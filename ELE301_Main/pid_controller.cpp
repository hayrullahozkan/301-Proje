#include "pid_controller.h"

PIDController::PIDController(float Ts) {
    _Ts = Ts;
    _Kp = 1.0;
    _Ki = 0.0;
    _Kd = 0.0;

    for (int i=0; i<4; i++) {
        _integral[i] = 0;
        _prevError[i] = 0;
    }
}

void PIDController::setGains(float Kp, float Ki, float Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

void PIDController::getGains(float &Kp, float &Ki, float &Kd) {
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
}

float PIDController::compute(float ref, float meas, int id) {
    float error = ref - meas;

    _integral[id] += error * _Ts;
    float derivative = (error - _prevError[id]) / _Ts;

    float u = _Kp*error + _Ki*_integral[id] + _Kd*derivative;

    _prevError[id] = error;

    // Saturation
    if (u > 255)  u = 255;
    if (u < -255) u = -255;

    return u;
}

void PIDController::reset() {
    for (int i=0;i<4;i++) {
        _integral[i] = 0;
        _prevError[i] = 0;
    }
}
