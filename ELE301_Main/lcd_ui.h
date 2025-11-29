#ifndef LCD_UI_H
#define LCD_UI_H

#include <Arduino.h>

// ---- Kontrol Modu ----
enum ControlMode {
    MODE_P = 0,
    MODE_PI,
    MODE_PD,
    MODE_PID
};

// ---- Başlat ----
void lcdUIInit();

// ---- Her loop'ta UI güncelle ----
void lcdUITask();

// ---- Mesafe LCD'ye yaz ----
void lcdSetDistance(int mm);

// ---- Değerleri main'e aktar ----
float getRefMm();
float getKp();
float getKi();
float getKd();
ControlMode getControlMode();
bool getAutoTuneEnabled();

// ---- Edit mode kontrol ----
bool lcdIsEditing();

// ---- Autotune güncel değerleri güncelle ----
void lcdUpdateGainsFromController(float kp, float ki, float kd);

#endif
