#ifndef LCD_UI_H
#define LCD_UI_H

#include <Arduino.h>

// Kontrol tipi (sadece yapı: P, PI, PD, PID)
enum ControlMode {
    MODE_P = 0,
    MODE_PI,
    MODE_PD,
    MODE_PID
};

// LCD + encoder UI başlatma
void lcdUIInit();

// Her loop turunda çağrılacak (encoder + buton + LCD menüsü)
void lcdUITask();

// Parametre getter'ları (UI tarafındaki DEĞERLER)
float getRefMm();          // Setpoint [mm]
float getKp();
float getKi();
float getKd();
ControlMode getControlMode();
bool getAutoTuneEnabled(); // Autotune ON/OFF

// Kullanıcı şu an edit modunda mı? (main'deki commit mantığı için)
bool lcdIsEditing();

// Sensörden gelen mesafeyi LCD'de göstermek için
void lcdSetDistance(int mm);

// Autotune açıkken, kontrolcünün anlık Kp/Ki/Kd değerlerini
// LCD'de göstermek için (edit modunda DEĞİLKEN günceller).
void lcdUpdateGainsFromController(float Kp, float Ki, float Kd);

#endif
