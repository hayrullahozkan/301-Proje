#include <Arduino.h>
#include "lcd_ui.h"
#include "vl53l0x.h"
#include "coils.h"
#include "pid_controller.h"
#include "pid_autotune.h"

// -------------------- ÖRNEKLEME --------------------
const float Ts      = 0.02f;        // 20 ms (50 Hz) kontrol döngüsü
unsigned long lastT = 0;

// -------------------- SENSÖR --------------------
VL53L0XDriver tof(7);               // XSHUT pini (gerekmiyorsa 255 bırak)

// -------------------- PID --------------------
PIDController pid(Ts);
PIDAutotune  autoTune(Ts);

// Aktif (commit edilmiş) parametreler
float       ref_mm       = 20.0f;
float       Kp_use       = 1.5f;
float       Ki_use       = 0.0f;
float       Kd_use       = 0.0f;
ControlMode mode_use     = MODE_PID;
bool        autoTune_use = false;

bool lastEditing = false;


// =======================================================
// SETUP
// =======================================================
void setup()
{
    Serial.begin(115200);
    delay(200);

    lcdUIInit();      // LCD + encoder
    coilsInit();      // 4 bobin sürücü çıkışları
    tof.begin();      // VL53L0X sensör

    // Autotune için fizik parametre örnekleri (gerektiğinde değiştir)
    float mass_g  = 15.0f;      // 15 gram mıknatıs
    float z0_cm   = 2.0f;       // denge yüksekliği ~ 2 cm
    float i0_A    = 0.8f;       // tahmini ortalama akım
    float g_val   = 9.81f;

    autoTune.setPhysicalParams(mass_g/1000.0f, z0_cm/100.0f, i0_A, g_val);
    autoTune.setDesiredDynamics(40.0f, 1.0f);   // wn=40 rad/s, zeta=1

    pid.setGains(Kp_use, Ki_use, Kd_use);
    pid.reset();

    Serial.println("=== ELE301 Maglev Basladi ===");
}


// =======================================================
// LOOP
// =======================================================
void loop()
{
    // ---------------------------------------------------
    // 1) LCD UI her döngüde çalıştırılır
    // ---------------------------------------------------
    lcdUITask();
    bool editingNow = lcdIsEditing();

    // ---------------------------------------------------
    // 2) Edit modundan çıkınca yeni parametreleri commit et
    // ---------------------------------------------------
    if (lastEditing && !editingNow)
    {
        ref_mm       = getRefMm();
        Kp_use       = getKp();
        Ki_use       = getKi();
        Kd_use       = getKd();
        mode_use     = getControlMode();
        autoTune_use = getAutoTuneEnabled();

        pid.setGains(Kp_use, Ki_use, Kd_use);
        pid.reset();
        autoTune.reset();
        autoTune.setMode(mode_use);

        Serial.println(">>> LCD'den yeni PID ayarlari commit edildi.");
    }

    lastEditing = editingNow;

    // ---------------------------------------------------
    // 3) Sensör oku + LCD'ye yaz
    // ---------------------------------------------------
    int z_raw = tof.readContinuous();   // mm
    float z_meas = (float)z_raw;
    lcdSetDistance(z_raw);

    if (z_raw < 0) {  // geçersiz veri
        coilsOff();
        return;
    }

    // ---------------------------------------------------
    // 4) PID sadece periyodik çalışsın
    // ---------------------------------------------------
    unsigned long now = millis();
    if (now - lastT < (unsigned long)(Ts*1000.0f))
        return;

    lastT = now;

    float ref = ref_mm;

    // ---------------------------------------------------
    // 5) Autotune açıksa yeni kazançları hesapla
    // ---------------------------------------------------
    float Kp = Kp_use;
    float Ki = Ki_use;
    float Kd = Kd_use;

    if (autoTune_use)
    {
        float Kp_t, Ki_t, Kd_t;
        autoTune.update(ref, z_meas, Kp_t, Ki_t, Kd_t);

        Kp = Kp_t;
        Ki = Ki_t;
        Kd = Kd_t;

        // Aktif değerleri güncelle (autotune kapandıktan sonra da devam etsin)
        Kp_use = Kp;
        Ki_use = Ki;
        Kd_use = Kd;

        // LCD'de güncelle (edit modunda değilse)
        lcdUpdateGainsFromController(Kp, Ki, Kd);
    }

    // ---------------------------------------------------
    // 6) Mod'a göre kazanç seçimleri (P, PI, PD, PID)
    // ---------------------------------------------------
    switch (mode_use)
    {
        case MODE_P:   Ki = 0;  Kd = 0; break;
        case MODE_PI:  Kd = 0; break;
        case MODE_PD:  Ki = 0; break;
        case MODE_PID: break;
    }

    pid.setGains(Kp, Ki, Kd);

    // ---------------------------------------------------
    // 7) TEK PID → TEK u_z
    // ---------------------------------------------------
    float u = pid.compute(ref, z_meas, 0);   // tek PID, tek integral

    // ---------------------------------------------------
    // 8) u değerini 4 bobine eşit uygula
    // ---------------------------------------------------
    coilSetPWM(COIL1, u);
    coilSetPWM(COIL2, u);
    coilSetPWM(COIL3, u);
    coilSetPWM(COIL4, u);

    // ---------------------------------------------------
    // 9) Debug
    // ---------------------------------------------------
    Serial.print("Z="); Serial.print(z_meas);
    Serial.print(" SP="); Serial.print(ref);
    Serial.print(" u="); Serial.print(u);
    Serial.print(" | Kp="); Serial.print(Kp);
    Serial.print(" Ki="); Serial.print(Ki);
    Serial.print(" Kd="); Serial.println(Kd);
}
