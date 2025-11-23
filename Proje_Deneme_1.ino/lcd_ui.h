// lcd_ui.h
#ifndef LCD_UI_H
#define LCD_UI_H

#include <Arduino.h>

// Kontrol modu tipleri
enum ModeType {
  MODE_P = 0,
  MODE_ONOFF,
  MODE_PD,
  MODE_PID,
  MODE_PI,
  MODE_COUNT
};

// ---- Dışarı açılan fonksiyonlar ----

// LCD + encoder + button arayüzünü başlat (setup'te 1 kez çağır)
void lcdUIInit();

// Her loop turunda çağrılacak task fonksiyonu
void lcdUITask();

// ---- Dışarıya parametre okuma fonksiyonları ----
float     getRefMm();        // Setpoint [mm] (10–40 arası)
float     getKp();
float     getKi();
float     getKd();
ModeType  getControlMode();  // P / ONOFF / PD / PID / PI

#endif // LCD_UI_H
