#include <Arduino.h>
#include "lcd_ui.h"
#include "vl53l0x.h"
#include "pid_controller.h"
#include "pid_autotune.h"
#include "coils.h"   // projede dursun, ama coilSetPWM çağrılmıyor

// -------------------------------------------------
// ZAMANLAMA
// -------------------------------------------------
const float Ts = 0.02f;       // 20ms PID periyodu
unsigned long lastPIDTime = 0;

// -------------------------------------------------
// SENSOR
// -------------------------------------------------
VL53L0XDriver tof(7);   // XSHUT = D7

// -------------------------------------------------
// PID
// -------------------------------------------------
PIDController pid(Ts);
PIDAutotune  autoTune(Ts);

// Aktif çalışan parametreler (edit kapanınca commit ediliyor)
float       ref_mm_active   = 20.0f;
float       Kp_active       = 1.50f;
float       Ki_active       = 0.00f;
float       Kd_active       = 0.00f;
ControlMode mode_active     = MODE_PID;
bool        autoTune_active = false;

// LCD - edit takip
bool lastEditing = false;


// -------------------------------------------------
// SETUP
// -------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(300);

    // LCD menü başlat
    lcdUIInit();

    // Sensörü başlat
    Serial.println("[INFO] Starting VL53L0X...");

    if (!tof.begin()) {
        Serial.println("[ERROR] VL53L0X init failed!");
        Serial.println("Check wiring:");
        Serial.println(" SDA -> D20");
        Serial.println(" SCL -> D21");
        Serial.println(" XSHUT -> D7");
        Serial.println(" VIN=5V, GND connected");
        while(1);
    }

    Serial.println("[OK] VL53L0X ready.");

    // --------------------------------------------
    // Fiziksel parametreler (SEN AYARLAYACAKSIN)
    // --------------------------------------------
    float mass_g  = 15.0f;
    float mass_kg = mass_g / 1000.0f;

    float z0_cm = 2.0f;
    float z0_m  = z0_cm / 100.0f;

    float i0_A  = 0.8f;
    float g_val = 9.81f;

    autoTune.setPhysicalParams(mass_kg, z0_m, i0_A, g_val);
    autoTune.setDesiredDynamics(40.0f, 1.0f); // wn = 40 rad/s

    // LCD’deki ilk değerleri al
    ref_mm_active   = getRefMm();
    Kp_active       = getKp();
    Ki_active       = getKi();
    Kd_active       = getKd();
    mode_active     = getControlMode();
    autoTune_active = getAutoTuneEnabled();

    pid.setGains(Kp_active, Ki_active, Kd_active);
    pid.reset();

    autoTune.reset();
    autoTune.setMode(mode_active);

    Serial.println("[SYSTEM] Ready. Coils disabled.");
}


// -------------------------------------------------
// LOOP
// -------------------------------------------------
void loop()
{
    // 1) LCD UI (encoder + button)
    lcdUITask();

    // 2) Edit kapanınca değerleri commit et
    bool editingNow = lcdIsEditing();
    if (lastEditing && !editingNow)
    {
        // yeni LCD değerlerini kontrolcüye yükle
        ref_mm_active   = getRefMm();
        Kp_active       = getKp();
        Ki_active       = getKi();
        Kd_active       = getKd();
        mode_active     = getControlMode();
        autoTune_active = getAutoTuneEnabled();

        pid.setGains(Kp_active, Ki_active, Kd_active);
        pid.reset();

        autoTune.reset();
        autoTune.setMode(mode_active);

        Serial.println(">>> UI VALUES COMMITTED <<<");
        Serial.print("SP   = "); Serial.println(ref_mm_active);
        Serial.print("Kp   = "); Serial.println(Kp_active);
        Serial.print("Ki   = "); Serial.println(Ki_active);
        Serial.print("Kd   = "); Serial.println(Kd_active);
        Serial.print("Mode = "); Serial.println((int)mode_active);
        Serial.print("AT   = "); Serial.println(autoTune_active);
    }
    lastEditing = editingNow;

    // 3) Sensörü oku
    int z_raw = tof.readContinuous();
    lcdSetDistance(z_raw);

    float z_meas = (float)z_raw;

    // 4) PID periyodu
    unsigned long now = millis();
    if (now - lastPIDTime >= (unsigned long)(Ts * 1000.0f))
    {
        lastPIDTime = now;

        float ref = ref_mm_active;

        float Kp_use = Kp_active;
        float Ki_use = Ki_active;
        float Kd_use = Kd_active;

        // --------------------------------------------
        // Autotune modu aktifse → kazançlar canlı hesaplanır
        // --------------------------------------------
        if (autoTune_active)
        {
            float Kp_t, Ki_t, Kd_t;
            autoTune.setMode(mode_active);
            autoTune.update(ref, z_meas, Kp_t, Ki_t, Kd_t);

            // Kullanılacak kazançlar
            Kp_use = Kp_t;
            Ki_use = Ki_t;
            Kd_use = Kd_t;

            // LCD’ye de yaz
            lcdUpdateGainsFromController(Kp_use, Ki_use, Kd_use);

            // şu anki aktif kazançlar güncellenmiş olur
            Kp_active = Kp_use;
            Ki_active = Ki_use;
            Kd_active = Kd_use;
        }

        // --------------------------------------------
        // MODE (P/PI/PD/PID)
        // --------------------------------------------
        switch(mode_active)
        {
            case MODE_P:   Ki_use = 0; Kd_use = 0; break;
            case MODE_PI:  Kd_use = 0; break;
            case MODE_PD:  Ki_use = 0; break;
            case MODE_PID: break;
        }

        // kontrolcüye kazançları ver
        pid.setGains(Kp_use, Ki_use, Kd_use);

        // 4 bobin için PID çıkışı (bobin sürülmeyecek)
        float u1 = pid.compute(ref, z_meas, 0);
        float u2 = pid.compute(ref, z_meas, 1);
        float u3 = pid.compute(ref, z_meas, 2);
        float u4 = pid.compute(ref, z_meas, 3);

        // COILLER KAPALI — GÜVENLİ SİMÜLASYON
        //coilSetPWM(COIL1, u1);
        //coilSetPWM(COIL2, u2);
        //coilSetPWM(COIL3, u3);
        //coilSetPWM(COIL4, u4);

        // -------- DEBUG --------
        Serial.print("Z="); Serial.print(z_meas);
        Serial.print(" SP="); Serial.print(ref);
        Serial.print(" Kp="); Serial.print(Kp_use,3);
        Serial.print(" Ki="); Serial.print(Ki_use,3);
        Serial.print(" Kd="); Serial.print(Kd_use,3);
        Serial.print(" AT="); Serial.print(autoTune_active ? "ON" : "OFF");
        Serial.print(" | u1="); Serial.print(u1);
        Serial.print(" u2="); Serial.print(u2);
        Serial.print(" u3="); Serial.print(u3);
        Serial.print(" u4="); Serial.println(u4);
    }
}
