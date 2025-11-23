#include <Wire.h>
#include "lcd_ui.h"
#include "coils.h"

const float Ts = 0.01f;          // 10 ms
unsigned long lastPIDTime   = 0;
unsigned long lastPrintTime = 0;
const unsigned long PRINT_PERIOD = 500; // ms

// Bitki çıktısı (yükseklik) – simülasyon
float z_mm = 20.0f;     // 🔹 Başlangıçta 20 mm

// PID durum değişkenleri (tek bobin için)
float integral  = 0.0f;
float prevError = 0.0f;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  coilsInit();
  lcdUIInit();

  // Gürültü için seed
  randomSeed(analogRead(A0));

  Serial.println(F("ELE301 Z-simulasyon demo basladi."));
}

void loop() {
  // 1) LCD + encoder + buton arayüzü
  lcdUITask();

  // 2) Kullanıcının seçtiği parametreleri al
  float   ref  = getRefMm();     // 10–40 mm
  float   Kp   = getKp();
  float   Ki   = getKi();
  float   Kd   = getKd();
  ModeType mode = getControlMode();

  // 3) Sabit periyotlu kontrol
  unsigned long now = millis();
  if (now - lastPIDTime >= (unsigned long)(Ts * 1000.0f)) {
    lastPIDTime = now;

    // ---- Hata ve PID hesap ----
    float error = ref - z_mm;
    integral   += error * Ts;
    float deriv = (error - prevError) / Ts;

    float u = 0.0f;

    switch (mode) {
      case MODE_P:
        u = Kp * error;
        break;

      case MODE_ONOFF:
        // Basit aç-kapa
        u = (error > 0.0f) ? 255.0f : -255.0f;
        break;

      case MODE_PD:
        u = Kp * error + Kd * deriv;
        break;

      case MODE_PI:
        u = Kp * error + Ki * integral;
        break;

      case MODE_PID:
        u = Kp * error + Ki * integral + Kd * deriv;
        break;
    }

    // Sınırla
    if (u >  255.0f) u =  255.0f;
    if (u < -255.0f) u = -255.0f;

    prevError = error;

    // ---- Bitki (coil + mıknatıs) simülasyonu ----
    //
    // Basit model: z(k+1) = z(k) + G*u*Ts + noise
    //  G: plantGain – coil akımı yüksek olunca z daha hızlı değişsin
    const float plantGain = 0.05f;         // deneme için
    float noise = (float)random(-3, 4) / 100.0f; // -0.03 .. +0.03 mm gürültü

    z_mm += plantGain * u * Ts + noise;

    // aralığı çok saçmalamasın diye kilitle
    if (z_mm < 0.0f)  z_mm = 0.0f;
    if (z_mm > 80.0f) z_mm = 80.0f;

    // 4) Hesaplanan u'yu 4 bobine de uygula
    coilSetPWM(COIL1, u);
    coilSetPWM(COIL2, u);
    coilSetPWM(COIL3, u);
    coilSetPWM(COIL4, u);

    // 5) Seri monitöre yaz (0.5 sn’de bir)
    if (now - lastPrintTime >= PRINT_PERIOD) {
      lastPrintTime = now;

      Serial.print(F("SP="));  Serial.print(ref);
      Serial.print(F(" mm, Z=")); Serial.print(z_mm, 2);
      Serial.print(F(" | Kp=")); Serial.print(Kp, 3);
      Serial.print(F(" Ki="));  Serial.print(Ki, 3);
      Serial.print(F(" Kd="));  Serial.print(Kd, 3);
      Serial.print(F(" | Mode="));

      switch (mode) {
        case MODE_P:     Serial.print(F("P"));      break;
        case MODE_ONOFF: Serial.print(F("ON/OFF")); break;
        case MODE_PD:    Serial.print(F("PD"));     break;
        case MODE_PID:   Serial.print(F("PID"));    break;
        case MODE_PI:    Serial.print(F("PI"));     break;
      }

      Serial.print(F(" | u=")); Serial.println(u, 2);
    }
  }
}
