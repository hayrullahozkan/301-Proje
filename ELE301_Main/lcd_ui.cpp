// lcd_ui.cpp
#include <Arduino.h>
#include <LiquidCrystal.h>
#include "lcd_ui.h"

// --- LCD pins (Arduino Mega 2560, Smart Controller EXP1) ---
#define LCD_RS 16
#define LCD_EN 17
#define LCD_D4 23
#define LCD_D5 25
#define LCD_D6 27
#define LCD_D7 29

static LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// --- Encoder & Button pins (RepRap Smart Controller, tipik eşleşme) ---
#define ENC_A   31   // Encoder A (BTN_EN1)
#define ENC_B   33   // Encoder B (BTN_EN2)
#define ENC_BTN 35   // Encoder push button (BTN_ENC)

// ---- İç state: kullanıcı parametreleri ----
static float ref_mm = 25.0f;  // Setpoint (10–40 mm arası)
static float Kp     = 1.50f;
static float Ki     = 0.10f;
static float Kd     = 0.05f;

static ModeType mode = MODE_PID;
int adim_miktari=2;//mm
// Hangi parametre seçili?
enum SelParam {SEL_SP = 0, SEL_KP, SEL_KI, SEL_KD, SEL_MODE, SEL_COUNT};
static SelParam sel = SEL_SP;

// Edit modu açık mı? (RUN / EDIT)
static bool editMode = false;

// Encoder durumu
static uint8_t lastEncState = 0;
static int     encAccum     = 0;   // küçük salınımları süzmek için toplama

// Button debounce
static const unsigned long BTN_DEBOUNCE = 30; // ms
static bool btnStableState   = HIGH;
static bool btnLastReading   = HIGH;
static unsigned long btnLastChangeTime = 0;

// Kontrol modu isimleri (LCD için)
static const char* modeNames[MODE_COUNT] = {
  "P",      // MODE_P
  "ONF",    // MODE_ONOFF (Aç-Kapa)
  "PD",     // MODE_PD
  "PID",    // MODE_PID
  "PI"      // MODE_PI
};


// ---------------------------------------------------
// Helper: LCD'de belli alana float yaz
// ---------------------------------------------------
static void lcdPrintFloat(float value, int col, int row, uint8_t prec = 2) {
  lcd.setCursor(col, row);
  lcd.print("       ");      // önce alanı temizle
  lcd.setCursor(col, row);
  lcd.print(value, prec);
}

// ---------------------------------------------------
// LCD ekranını güncelle
// ---------------------------------------------------
static void updateLCD() {
  lcd.clear();

  // -------- Satir 0: SP --------
  lcd.setCursor(0, 0);
  if (sel == SEL_SP) lcd.print(editMode ? ">>" : "> ");
  else               lcd.print("  ");
  lcd.print("SP:");
  lcdPrintFloat(ref_mm, 5, 0, 1);
  lcd.setCursor(13, 0);
  lcd.print("mm");

  // Sağ üstte RUN/EDIT göstergesi
  lcd.setCursor(17, 0);
  if (editMode) lcd.print("E");  // Edit
  else          lcd.print("R");  // Run

  // -------- Satir 1: Kp --------
  lcd.setCursor(0, 1);
  if (sel == SEL_KP) lcd.print(editMode ? ">>" : "> ");
  else               lcd.print("  ");
  lcd.print("Kp:");
  lcdPrintFloat(Kp, 5, 1, 3);   // 0.005 adım için 3 hane

  // -------- Satir 2: Ki --------
  lcd.setCursor(0, 2);
  if (sel == SEL_KI) lcd.print(editMode ? ">>" : "> ");
  else               lcd.print("  ");
  lcd.print("Ki:");
  lcdPrintFloat(Ki, 5, 2, 3);

  // -------- Satir 3: Kd ve Mode --------
  // Kd solda:
  lcd.setCursor(0, 3);
  if (sel == SEL_KD) lcd.print(editMode ? ">>" : "> ");
  else               lcd.print("  ");
  lcd.print("Kd:");
  lcdPrintFloat(Kd, 5, 3, 3);

  // Mode sağ tarafta:
  lcd.setCursor(12, 3);
  if (sel == SEL_MODE) {
    lcd.print(editMode ? ">>" : "> ");
  } else {
    lcd.print("  ");
  }
  lcd.print("Md:");
  lcd.print(modeNames[mode]);  // P / ONF / PD / PID / PI
}

// ---------------------------------------------------
// Encoder Okuma
// ---------------------------------------------------
static void readEncoder() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  uint8_t encState = (a << 1) | b;

  if (encState == lastEncState) return;  // değişim yok

  uint8_t index = (lastEncState << 2) | encState;

  // Quadrature transition table
  const int8_t encTable[16] = {
    0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
  };

  int8_t delta = encTable[index];
  lastEncState = encState;

  if (delta == 0) return;

  // küçük salınımları filtrelemek için biriktir
  encAccum += delta;

  int step = 0;
  if (encAccum >= 4) {      // 4 "tik" -> 1 step
    step = +1;
    encAccum = 0;
  } else if (encAccum <= -4) {
    step = -1;
    encAccum = 0;
  }

  if (step == 0) return;    // henüz yeterli hareket yok

  if (!editMode) {
    // --- RUN MODU: sadece hangi parametrenin seçili olduğunu değiştir ---
    int s = (int)sel + step;
    if (s < 0)           s = SEL_COUNT - 1;
    if (s >= SEL_COUNT)  s = 0;
    sel = (SelParam)s;
  } else {
    // --- EDIT MODU: seçili parametrenin değerini / Mode'u değiştir ---
    switch (sel) {
      case SEL_SP:
        // SP: 5 mm adım, 10–40 mm aralığı
        ref_mm += step * adim_miktari;
        if (ref_mm < 10.0f) ref_mm = 10.0f;
        if (ref_mm > 40.0f) ref_mm = 40.0f;
        break;

      case SEL_KP:
        // Kp: 0.005 adım
        Kp += step * 0.005f;
        if (Kp < 0.0f) Kp = 0.0f;
        break;

      case SEL_KI:
        // Ki: 0.005 adım
        Ki += step * 0.005f;
        if (Ki < 0.0f) Ki = 0.0f;
        break;

      case SEL_KD:
        // Kd: 0.005 adım
        Kd += step * 0.005f;
        if (Kd < 0.0f) Kd = 0.0f;
        break;

      case SEL_MODE:
      {
        int m = (int)mode + step;
        if (m < 0)           m = MODE_COUNT - 1;
        if (m >= MODE_COUNT) m = 0;
        mode = (ModeType)m;
        break;
      }
    }
  }

  updateLCD();
}

// ---------------------------------------------------
// Button Okuma: RUN <-> EDIT geçişi (debounce'lu)
// ---------------------------------------------------
static void readButton() {
  int reading = digitalRead(ENC_BTN);
  unsigned long now = millis();

  if (reading != btnLastReading) {
    btnLastChangeTime = now;
    btnLastReading    = reading;
  }

  if ((now - btnLastChangeTime) > BTN_DEBOUNCE) {
    if (reading != btnStableState) {
      btnStableState = reading;

      // HIGH -> LOW geçişi
      if (btnStableState == LOW) {
        editMode = !editMode;
        updateLCD();
      }
    }
  }
}


// ---------------------------------------------------
// DIŞA AÇILAN FONKSİYONLAR
// ---------------------------------------------------

void lcdUIInit() {
  lcd.begin(20, 4);
  lcd.clear();

  pinMode(ENC_A,   INPUT_PULLUP);
  pinMode(ENC_B,   INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);

  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  lastEncState   = (a << 1) | b;

  btnStableState    = digitalRead(ENC_BTN);
  btnLastReading    = btnStableState;
  btnLastChangeTime = millis();

  updateLCD();
}

void lcdUITask() {
  readEncoder();
  readButton();
}

// Getter'lar
float getRefMm()          { return ref_mm; }
float getKp()             { return Kp;     }
float getKi()             { return Ki;     }
float getKd()             { return Kd;     }
ModeType getControlMode() { return mode;   }
