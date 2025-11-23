#ifndef PID_AUTOTUNE_H
#define PID_AUTOTUNE_H

#include <Arduino.h>
#include "lcd_ui.h"   // ControlMode enum'u için

class PIDAutotune {
public:
    PIDAutotune(float Ts);

    void reset();
    void setMode(ControlMode mode);

    // ⚠️ Bunlar yeni fonksiyonlar — .ino bunları çağırıyor
    void setPhysicalParams(float mass_kg, float z0_m, float i0_A, float g_val = 9.81f);
    void setDesiredDynamics(float wn, float zeta = 1.0f);

    // Autotune adımı (çıkış olarak yeni Kp/Ki/Kd üretir)
    void update(float ref, float meas,
                float &Kp_out, float &Ki_out, float &Kd_out);

private:
    float _Ts;
    ControlMode _mode;

    // Kullanıcının girdiği fiziksel parametreler
    float _m;      // kg
    float _z0;     // m
    float _i0;     // A
    float _g;      // m/s^2

    // Hedef kapalı çevrim (model tabanlı autotune)
    float _wn;
    float _zeta;

    // Bu adımda hesaplanan kazançlar
    float _Kp;
    float _Ki;
    float _Kd;
};

#endif
