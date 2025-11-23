#include <Arduino.h>
#include "lcd_ui.h"         // SP, Kp, Ki, Kd, Mode, AutoTune, lcdIsEditing
#include "vl53l0x.h"        // VL53L0XDriver
#include "coils.h"          // coilsInit, coilSetPWM, coilsOff
#include "pid_controller.h" // PIDController
#include "pid_autotune.h"   // PIDAutotune (fiziksel parametreli)

// -------------------- ZAMANLAMA --------------------
const float Ts = 0.02f;              // 20 ms kontrol periyodu
unsigned long lastPIDTime = 0;

// -------------------- SENSÖR --------------------
VL53L0XDriver tof(7);                // XSHUT = D7 (istersen değiştir)

// -------------------- PID CONTROLLER --------------------
PIDController pid(Ts);
PIDAutotune  autoTune(Ts);

// Controlcüde kullanılan AKTİF (commit edilmiş) parametreler
float       ref_mm_active   = 20.0f;
float       Kp_active       = 1.50f;
float       Ki_active       = 0.00f;
float       Kd_active       = 0.00f;
ControlMode mode_active     = MODE_PID;
bool        autoTune_active = false;

// LCD edit durumu takibi (commit için)
bool lastEditing = false;


// -------------------- SETUP --------------------
void setup()
{
    Serial.begin(115200);
    delay(300);

    // LCD + encoder + buton
    lcdUIInit();

    // Bobin sürücüleri (L298N)
    coilsInit();

    // Mesafe sensörü
    tof.begin();

    // ---------------- FİZİKSEL PARAMETRELERİ BURADAN AYARLAYACAKSIN ----------------
    // ÖRNEK: 15 gram mıknatıs, çalışma yüksekliği 2 cm, denge akımı 0.8 A
    float mass_g  = 15.0f;     // gram cinsinden; sen gerçek değeri yaz
    float mass_kg = mass_g / 1000.0f;

    float z0_cm = 2.0f;        // cm cinsinden denge yüksekliği; sen gerçek değeri yaz
    float z0_m  = z0_cm / 100.0f;

    float i0_A  = 0.8f;        // denge akımı [A]; sen gerçek değeri yaz
    float g_val = 9.81f;       // yerçekimi

    autoTune.setPhysicalParams(mass_kg, z0_m, i0_A, g_val);

    // Hedef kapalı çevrim hızı (doğal frekans, rad/s):
    // 20–60 rad/s arası bir şey deneyebilirsin (hızlı tepki için yüksek)
    autoTune.setDesiredDynamics(40.0f, 1.0f);   // wn = 40 rad/s, zeta=1

    // Başlangıçta LCD'deki değerleri ilk kez commit et
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

    Serial.println(F("ELE301 MAIN STARTED"));
}


// -------------------- LOOP --------------------
void loop()
{
    // 1) LCD / Encoder / Buton UI'yi işlet
    lcdUITask();

    // 2) Kullanıcının şu an ayar (edit) modunda olup olmadığını kontrol et
    bool editingNow = lcdIsEditing();

    // 3) Edit modundan ÇIKIŞ anını tespit et (commit olayı)
    //
    // lastEditing = true, editingNow = false ise:
    //   Kullanıcı az önce edit modundan çıktı -> yeni değerleri PID'e uygula
    //
    if (lastEditing && !editingNow) {
        // --- Yeni LCD değerlerini CONTROL tarafına commit et ---
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

        Serial.println(F(">>> NEW PARAMETERS COMMITTED FROM LCD <<<"));
        Serial.print(F("SP="));  Serial.print(ref_mm_active);
        Serial.print(F(" Kp=")); Serial.print(Kp_active, 3);
        Serial.print(F(" Ki=")); Serial.print(Ki_active, 3);
        Serial.print(F(" Kd=")); Serial.print(Kd_active, 3);
        Serial.print(F(" Mode=")); Serial.print((int)mode_active);
        Serial.print(F(" AT="));   Serial.println(autoTune_active ? "ON" : "OFF");
    }

    // editing durumunu bir sonraki tur için sakla
    lastEditing = editingNow;

    // 4) Sensörü oku ve LCD'de göster
    int z_raw = tof.readContinuous();   // mm cinsinden
    lcdSetDistance(z_raw);
    float z_meas = (float)z_raw;

    // 5) Sabit periyotlu PID kontrol
    unsigned long now = millis();
    if (now - lastPIDTime >= (unsigned long)(Ts * 1000.0f)) {
        lastPIDTime = now;

        float ref = ref_mm_active;

        // Başlangıç olarak: aktif (commit edilmiş) kazançlar
        float Kp_use = Kp_active;
        float Ki_use = Ki_active;
        float Kd_use = Kd_active;

        // --- Autotune Açık mı? ---
        if (autoTune_active) {
            float Kp_t, Ki_t, Kd_t;
            autoTune.setMode(mode_active);  // P / PI / PD / PID
            autoTune.update(ref, z_meas, Kp_t, Ki_t, Kd_t);

            // Autotune her adımda Kp/Ki/Kd'yi güncelliyor
            Kp_use = Kp_t;
            Ki_use = Ki_t;
            Kd_use = Kd_t;

            // Aktif değerleri de güncelle ki,
            // AutoTune kapatıldığında bu yeni kazançlarla devam edilsin.
            Kp_active = Kp_use;
            Ki_active = Ki_use;
            Kd_active = Kd_use;

            // LCD'de de bu yeni kazançlar görünsün (edit modunda DEĞİLSE)
            lcdUpdateGainsFromController(Kp_active, Ki_active, Kd_active);
        }

        // --- Kontrol tipine göre kullanılacak kazançları ayarla ---
        switch (mode_active) {
            case MODE_P:
                Ki_use = 0.0f;
                Kd_use = 0.0f;
                break;

            case MODE_PI:
                Kd_use = 0.0f;
                break;

            case MODE_PD:
                Ki_use = 0.0f;
                break;

            case MODE_PID:
                // Hepsi aktif
                break;
        }

        // Controlcüye güncel kazançları ver
        pid.setGains(Kp_use, Ki_use, Kd_use);

        // 4 bobin için aynı referans ve ölçümle ayrı ID'ler kullanıyoruz
        float u1 = pid.compute(ref, z_meas, 0);
        float u2 = pid.compute(ref, z_meas, 1);
        float u3 = pid.compute(ref, z_meas, 2);
        float u4 = pid.compute(ref, z_meas, 3);

        // Bobinleri sür
        coilSetPWM(COIL1, u1);
        coilSetPWM(COIL2, u2);
        coilSetPWM(COIL3, u3);
        coilSetPWM(COIL4, u4);

        // Debug
        Serial.print(F("Z="));  Serial.print(z_meas);
        Serial.print(F(" SP=")); Serial.print(ref);
        Serial.print(F(" Kp=")); Serial.print(Kp_use, 3);
        Serial.print(F(" Ki=")); Serial.print(Ki_use, 3);
        Serial.print(F(" Kd=")); Serial.print(Kd_use, 3);
        Serial.print(F(" AT=")); Serial.print(autoTune_active ? "ON" : "OFF");
        Serial.print(F(" | u1=")); Serial.print(u1);
        Serial.print(F(" u2="));  Serial.print(u2);
        Serial.print(F(" u3="));  Serial.print(u3);
        Serial.print(F(" u4="));  Serial.println(u4);
    }
}
