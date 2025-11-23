// coils.cpp
#include "coils.h"

// -------------------------
// Pin haritaları
// -------------------------
//
// Bobin 1 -> L298N #1 OUT1-OUT2
//   EN: D2   (PWM)
//   IN1: D22
//   IN2: D26
//
// Bobin 2 -> L298N #1 OUT3-OUT4
//   EN: D4   (PWM)
//   IN1: D28
//   IN2: D30
//
// Bobin 3 -> L298N #2 OUT1-OUT2
//   EN: D6   (PWM)
//   IN1: D24
//   IN2: D32
//
// Bobin 4 -> L298N #2 OUT3-OUT4
//   EN: D8   (PWM)
//   IN1: D34
//   IN2: D36
//

static const uint8_t EN_PINS[4]  = {2, 4, 6, 8};
static const uint8_t IN1_PINS[4] = {22, 28, 24, 34};
static const uint8_t IN2_PINS[4] = {26, 30, 32, 36};


// ---------------------------------------------------
// Başlatma
// ---------------------------------------------------
void coilsInit() {
  for (int i = 0; i < 4; ++i) {
    pinMode(EN_PINS[i],  OUTPUT);
    pinMode(IN1_PINS[i], OUTPUT);
    pinMode(IN2_PINS[i], OUTPUT);

    // Hepsini kapalı başlat
    analogWrite(EN_PINS[i], 0);
    digitalWrite(IN1_PINS[i], LOW);
    digitalWrite(IN2_PINS[i], LOW);
  }
}

// ---------------------------------------------------
// Tek bobin sürme
// u: -255 .. +255  (PID çıkışını direkt verebilirsin)
// ---------------------------------------------------
void coilSetPWM(CoilId id, float u) {
  if (id < COIL1 || id > COIL4) return;

  // sınırla
  if (u >  255.0f) u =  255.0f;
  if (u < -255.0f) u = -255.0f;

  uint8_t idx = (uint8_t)id;
  int duty;

  if (u >= 0.0f) {
    // Bir yönde akım
    digitalWrite(IN1_PINS[idx], HIGH);
    digitalWrite(IN2_PINS[idx], LOW);
    duty = (int)(u + 0.5f);      // yuvarla
  } else {
    // Ters yönde akım
    digitalWrite(IN1_PINS[idx], LOW);
    digitalWrite(IN2_PINS[idx], HIGH);
    duty = (int)(-u + 0.5f);
  }

  analogWrite(EN_PINS[idx], duty);
}

// ---------------------------------------------------
// Tüm bobinleri kapat
// ---------------------------------------------------
void coilsOff() {
  for (int i = 0; i < 4; ++i) {
    analogWrite(EN_PINS[i], 0);
    digitalWrite(IN1_PINS[i], LOW);
    digitalWrite(IN2_PINS[i], LOW);
  }
}
