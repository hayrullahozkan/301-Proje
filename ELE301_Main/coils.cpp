#include "coils.h"

// ---- PIN DİZİLERİNİN GERÇEK TANIMLARI ----
// Header'daki extern bildirimleri bunlara bağlanır.
// static kaldırıldı → tüm proje bunları görebilir.
int pinEn[4]  = {2, 4, 6, 8};
int pinIn1[4] = {22, 28, 24, 34};
int pinIn2[4] = {26, 30, 32, 36};

// ---------------------------------------------------
// Başlatma (pinMode + kapalı başlat)
// ---------------------------------------------------
void coilsInit() {
  for (int i = 0; i < 4; ++i) {
    pinMode(pinEn[i],  OUTPUT);
    pinMode(pinIn1[i], OUTPUT);
    pinMode(pinIn2[i], OUTPUT);

    analogWrite(pinEn[i], 0);
    digitalWrite(pinIn1[i], LOW);
    digitalWrite(pinIn2[i], LOW);
  }
}

// ---------------------------------------------------
// Tek bobin sürme
// ---------------------------------------------------
void coilSetPWM(CoilId id, float u) {
  if (id < COIL1 || id > COIL4) return;

  if (u >  255) u =  255;
  if (u < -255) u = -255;

  uint8_t i = (uint8_t)id;
  int duty = abs((int)u);

  if (u >= 0) {
    digitalWrite(pinIn1[i], HIGH);
    digitalWrite(pinIn2[i], LOW);
  } else {
    digitalWrite(pinIn1[i], LOW);
    digitalWrite(pinIn2[i], HIGH);
  }

  analogWrite(pinEn[i], duty);
}

// ---------------------------------------------------
// Tüm bobinleri kapat
// ---------------------------------------------------
void coilsOff() {
  for (int i = 0; i < 4; ++i) {
    analogWrite(pinEn[i], 0);
    digitalWrite(pinIn1[i], LOW);
    digitalWrite(pinIn2[i], LOW);
  }
}
