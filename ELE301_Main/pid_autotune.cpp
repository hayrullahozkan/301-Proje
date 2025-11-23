#include "pid_autotune.h"
#include <math.h>

PIDAutotune::PIDAutotune(float Ts)
{
    _Ts   = Ts;
    _mode = MODE_PID;

    // Varsayılan fiziksel parametreler (MAIN bunları override edecek)
    _m  = 0.02f;   // 20 gram = 0.02 kg
    _z0 = 0.02f;   // 2 cm = 0.02 m
    _i0 = 1.0f;    // 1 A
    _g  = 9.81f;

    // Varsayılan kapalı çevrim (MAIN override edecek)
    _wn   = 40.0f; 
    _zeta = 1.0f;

    _Kp = 1.0f;
    _Ki = 0.0f;
    _Kd = 0.0f;
}

void PIDAutotune::reset()
{
    // Şimdilik bir state tutulmuyor
}

void PIDAutotune::setMode(ControlMode mode)
{
    _mode = mode;
}

void PIDAutotune::setPhysicalParams(float mass_kg, float z0_m, float i0_A, float g_val)
{
    _m  = mass_kg;
    _z0 = z0_m;
    _i0 = i0_A;
    _g  = g_val;
}

void PIDAutotune::setDesiredDynamics(float wn, float zeta)
{
    _wn   = wn;
    _zeta = zeta;
}

void PIDAutotune::update(float ref, float meas,
                         float &Kp_out, float &Ki_out, float &Kd_out)
{
    if (_z0 <= 0.0f || _i0 <= 0.0f) {
        Kp_out = 0;
        Ki_out = 0;
        Kd_out = 0;
        return;
    }

    // Sistem model parametreleri:
    // G(s) = (2g/i0) / (s² + 2g/z0)
    float a = 2.0f * _g / _z0;
    float K = 2.0f * _g / _i0;

    // Hedef kapalı çevrim (s + wn)³ = s³ + 3wn s² + 3wn² s + wn³
    float w = _wn;

    float alpha2 = 3.0f * w;
    float alpha1 = 3.0f * w * w;
    float alpha0 = w * w * w;

    float Kp_calc = (alpha1 - a) / K;
    float Ki_calc = alpha0 / K;
    float Kd_calc = alpha2 / K;

    // Mode'a göre aktif kazançlar
    switch (_mode) {
        case MODE_P:
            Ki_calc = 0;
            Kd_calc = 0;
            break;
        case MODE_PI:
            Kd_calc = 0;
            break;
        case MODE_PD:
            Ki_calc = 0;
            break;
        case MODE_PID:
            break;
    }

    // Kaydet
    _Kp = Kp_calc;
    _Ki = Ki_calc;
    _Kd = Kd_calc;

    // Çıkışlara yaz
    Kp_out = _Kp;
    Ki_out = _Ki;
    Kd_out = _Kd;
}
