#include <Arduino.h>
#include "lcd_ui.h"
#include "vl53l0x.h"
#include "coils.h"
#include "pid_controller.h"
#include "pid_autotune.h"

// -------------------- SAMPLE TIME --------------------
const float Ts      = 0.02f;     // 20 ms (50 Hz)
unsigned long lastT = 0;

// -------------------- SENSOR --------------------
VL53L0XDriver tof(7);            // XSHUT pini (kullanmıyorsan 255)

// -------------------- PID --------------------
PIDController pid(Ts);
PIDAutotune  autoTune(Ts);

// Commit edilmiş (aktif) parametreler
float       ref_mm       = 20.0f;
float       Kp_active    = 1.50f;
float       Ki_active    = 0.00f;
float       Kd_active    = 0.00f;
ControlMode mode_active  = MODE_PID;
bool        autoTune_on  = false;

bool lastEditing = false;


// =======================================================
// SETUP
// =======================================================
void setup()
{
    Serial.begin(115200);
    delay(200);

    lcdUIInit();
    coilsInit();

    if (!tof.begin()) {
        Serial.println("VL53L0X ERROR. Check wiring!");
        while (1);
    }

    // ---------------- AUTOTUNE PHYSICAL PARAMS ----------------
    float mass_g  = 15.0f;
    float z0_cm   = 2.0f;
    float i0_A    = 0.8f;
    float g_val   = 9.81f;

    autoTune.setPhysicalParams(
        mass_g / 1000.0f,
        z0_cm / 100.0f,
        i0_A,
        g_val
    );

    autoTune.setDesiredDynamics(40.0f, 1.0f);

    // İlk LCD değerlerini al
    ref_mm      = getRefMm();
    Kp_active   = getKp();
    Ki_active   = getKi();
    Kd_active   = getKd();
    mode_active = getControlMode();
    autoTune_on = getAutoTuneEnabled();

    pid.setGains(Kp_active, Ki_active, Kd_active);
    pid.reset();

    autoTune.reset();
    autoTune.setMode(mode_active);

    Serial.println("=== MAGLEV SYSTEM READY ===");
}


// =======================================================
// LOOP
// =======================================================
void loop()
{
    // ---------------------------------------------------
    // 1) UI (LCD + Encoder)
    // ---------------------------------------------------
    lcdUITask();
    bool editingNow = lcdIsEditing();

    // ---------------------------------------------------
    // 2) Edit modundan çıkınca (commit)
    // ---------------------------------------------------
    if (lastEditing && !editingNow)
    {
        ref_mm      = getRefMm();
        Kp_active   = getKp();
        Ki_active   = getKi();
        Kd_active   = getKd();
        mode_active = getControlMode();
        autoTune_on = getAutoTuneEnabled();

        pid.setGains(Kp_active, Ki_active, Kd_active);
        pid.reset();
        autoTune.reset();
        autoTune.setMode(mode_active);

        Serial.println(">>> NEW PARAMETERS COMMITTED <<<");
    }

    lastEditing = editingNow;

    // ---------------------------------------------------
    // 3) Sensör oku
    // ---------------------------------------------------
    int z_raw = tof.readContinuous();
    lcdSetDistance(z_raw);

    if (z_raw < 0) {
        coilsOff();
        return;
    }

    float z_meas = (float)z_raw;

    // ---------------------------------------------------
    // 4) PID periyodik olsun
    // ---------------------------------------------------
    unsigned long now = millis();
    if (now - lastT < (unsigned long)(Ts * 1000.0f))
        return;

    lastT = now;

    float ref = ref_mm;

    // ---------------------------------------------------
    // 5) Autotune aktifse kazançları güncelle
    // ---------------------------------------------------
    float Kp = Kp_active;
    float Ki = Ki_active;
    float Kd = Kd_active;

    if (autoTune_on)
    {
        float Kp_t, Ki_t, Kd_t;
        autoTune.update(ref, z_meas, Kp_t, Ki_t, Kd_t);

        Kp = Kp_t;
        Ki = Ki_t;
        Kd = Kd_t;

        // LCD’ye yaz
        lcdUpdateGainsFromController(Kp, Ki, Kd);

        // commit et
        Kp_active = Kp;
        Ki_active = Ki;
        Kd_active = Kd;
    }

    // ---------------------------------------------------
    // 6) P / PI / PD / PID seçimleri
    // ---------------------------------------------------
    switch (mode_active)
    {
        case MODE_P:
            Ki = 0.0f;
            Kd = 0.0f;
            break;

        case MODE_PI:
            Kd = 0.0f;
            break;

        case MODE_PD:
            Ki = 0.0f;
            break;

        case MODE_PID:
            break;
    }

    pid.setGains(Kp, Ki, Kd);

    // ---------------------------------------------------
    // 7) PID → tek u_z
    // ---------------------------------------------------
    float u = pid.compute(ref, z_meas, 0);

    // ---------------------------------------------------
    // 8) 4 bobine aynı u_z
    // ---------------------------------------------------
    coilSetPWM(COIL1, u);
    coilSetPWM(COIL2, u);
    coilSetPWM(COIL3, u);
    coilSetPWM(COIL4, u);

    // ---------------------------------------------------
    // 9) Debug
    // ---------------------------------------------------
    Serial.print("Z=");  Serial.print(z_meas);
    Serial.print(" SP="); Serial.print(ref);
    Serial.print(" u="); Serial.print(u);
    Serial.print(" | Kp="); Serial.print(Kp);
    Serial.print(" Ki="); Serial.print(Ki);
    Serial.print(" Kd="); Serial.println(Kd);



    // ---------------------------------------------------
    // 9.5) Coil Debug (bobinlere giden gerçek sinyal)
    // ---------------------------------------------------
    Serial.print("[COILS] u="); Serial.print(u);

    // Coil 1
    Serial.print(" | C1_IN1="); Serial.print(digitalRead(pinIn1[COIL1]));
    Serial.print(" C1_IN2=");   Serial.print(digitalRead(pinIn2[COIL1]));
    Serial.print(" C1_PWM=");   Serial.print(analogRead(pinEn[COIL1]));

    // Coil 2
    Serial.print(" | C2_IN1="); Serial.print(digitalRead(pinIn1[COIL2]));
    Serial.print(" C2_IN2=");   Serial.print(digitalRead(pinIn2[COIL2]));
    Serial.print(" C2_PWM=");   Serial.print(analogRead(pinEn[COIL2]));

    // Coil 3
    Serial.print(" | C3_IN1="); Serial.print(digitalRead(pinIn1[COIL3]));
    Serial.print(" C3_IN2=");   Serial.print(digitalRead(pinIn2[COIL3]));
    Serial.print(" C3_PWM=");   Serial.print(analogRead(pinEn[COIL3]));

    // Coil 4
    Serial.print(" | C4_IN1="); Serial.print(digitalRead(pinIn1[COIL4]));
    Serial.print(" C4_IN2=");   Serial.print(digitalRead(pinIn2[COIL4]));
    Serial.print(" C4_PWM=");   Serial.print(analogRead(pinEn[COIL4]));

    Serial.println();
}
