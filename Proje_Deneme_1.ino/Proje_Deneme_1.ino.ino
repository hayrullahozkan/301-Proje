/****************************************************
   ELE 301 — 4 Bobin + PID + VL53L0X + 4×4 Keypad Menü
   + 20x4 Reprap Smart LCD (RAMPS 1.4)
****************************************************/

#include <Wire.h>
#include <VL53L0X.h>
#include <Keypad.h>
#include <LiquidCrystal.h>

VL53L0X sensor;

// ---------------- PID Parametreleri ----------------
float Kp[4] = {1.5, 1.5, 1.5, 1.5};
float Ki[4] = {0.0, 0.0, 0.0, 0.0};
float Kd[4] = {0.0, 0.0, 0.0, 0.0};

float integral[4]   = {0,0,0,0};
float prevError[4]  = {0,0,0,0};
float derivative[4] = {0,0,0,0};

float Ts = 0.01;  // 10 ms
unsigned long lastTime = 0;


// ---------------------------------------------------
// L298N Pinleri
// ---------------------------------------------------

// L298N #1
int EN1   = 2;   // PWM
int IN1_1 = 22;
int IN1_2 = 26;

int EN2   = 4;
int IN2_1 = 28;
int IN2_2 = 30;

// L298N #2
int EN3   = 6;
int IN3_1 = 24;
int IN3_2 = 32;

int EN4   = 8;
int IN4_1 = 34;
int IN4_2 = 36;


// ---------------------------------------------------
// Keypad Tanımı
// ---------------------------------------------------

const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {40, 42, 44, 46};
byte colPins[COLS] = {48, 50, 52, 53};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);


// ---------------------------------------------------
// LCD Tanımı (RAMPS 1.4 Smart Controller 2004)
// ---------------------------------------------------
// RS=16, EN=17, D4=23, D5=25, D6=27, D7=29

#define LCD_RS 16
#define LCD_EN 17
#define LCD_D4 23
#define LCD_D5 25
#define LCD_D6 27
#define LCD_D7 29

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);


// ---------------------------------------------------
// Menü Değişkenleri
// ---------------------------------------------------

enum Mode {MODE_P, MODE_PI, MODE_PD, MODE_PID};
Mode mode = MODE_PID;   // varsayılan PID

enum Param {NONE, SETPOINT, KP_, KI_, KD_};
Param currentParam = NONE;

// ondalıklı değer girişi
float currentValue    = 0.0;
bool  enteringValue   = false;
bool  hasDecimal      = false;
float decimalFactor   = 1.0;   // 0.1, 0.01, ...

// Setpoint (mm)
float ref_mm = 50.0;

// Son ölçülen yükseklik (mm)
float lastZ = 0.0;

// LCD yanıp sönme kontrolü
unsigned long lastBlinkTime = 0;
bool blinkOn = true;
const unsigned long BLINK_PERIOD = 400; // ms


// ---------------------------------------------------
// VL53L0X Başlatma
// ---------------------------------------------------
void initToF() {
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous(10);   // 10 ms ölçüm → PID ile uyumlu
}


// ---------------------------------------------------
// Menü Yazdırma (Seri Port)
// ---------------------------------------------------
void showMenuSerial() {
  Serial.println(F("------ MENU ------"));
  Serial.println(F("1 -> Setpoint (mm) gir   (D ile onay, * = . )"));
  Serial.println(F("2 -> Kp gir              (D ile onay, * = . )"));
  Serial.println(F("3 -> Ki gir              (D ile onay, * = . )"));
  Serial.println(F("4 -> Kd gir              (D ile onay, * = . )"));
  Serial.println(F("A -> P   ,  B -> PI"));
  Serial.println(F("C -> PD  ,  # -> PID"));
  Serial.println(F("* -> Menüyü yazdir (deger girilmiyorken)"));
  Serial.println(F("C (deger girerken) -> input clear"));
  Serial.println(F("D (deger girerken) -> onayla"));
  Serial.println(F("-------------------"));
}


// ---------------------------------------------------
// Input temizleme
// ---------------------------------------------------
void clearInput() {
  currentValue  = 0.0;
  hasDecimal    = false;
  decimalFactor = 1.0;
  Serial.println(F("Input cleared."));
}


// ---------------------------------------------------
// Değer Onaylama
// ---------------------------------------------------
void applyValue() {
  switch (currentParam) {

    case SETPOINT:
      ref_mm = currentValue;
      Serial.print(F("Yeni Setpoint (mm) = "));
      Serial.println(ref_mm, 3);
      break;

    case KP_:
      for (int i = 0; i < 4; i++) Kp[i] = currentValue;
      Serial.print(F("Yeni Kp = "));
      Serial.println(currentValue, 3);
      break;

    case KI_:
      for (int i = 0; i < 4; i++) Ki[i] = currentValue;
      Serial.print(F("Yeni Ki = "));
      Serial.println(currentValue, 3);
      break;

    case KD_:
      for (int i = 0; i < 4; i++) Kd[i] = currentValue;
      Serial.print(F("Yeni Kd = "));
      Serial.println(currentValue, 3);
      break;

    case NONE:
    default:
      break;
  }

  enteringValue = false;
  currentParam  = NONE;
  clearInput();
}


// ---------------------------------------------------
// LCD'de belli bir alana float yaz (genişlik sabitlemek için)
// ---------------------------------------------------
void lcdPrintFloat(float value, int col, int row, uint8_t prec = 2) {
  lcd.setCursor(col, row);
  lcd.print("       ");       // eskiyi sil
  lcd.setCursor(col, row);
  lcd.print(value, prec);
}


// ---------------------------------------------------
// LCD Güncelleme
// ---------------------------------------------------
void updateLCD() {

  // Yanıp sönme durumu
  if (enteringValue) {
    unsigned long now = millis();
    if (now - lastBlinkTime >= BLINK_PERIOD) {
      lastBlinkTime = now;
      blinkOn = !blinkOn;
    }
  } else {
    blinkOn = true;
  }

  // ------------- Satır 0: SP -------------
  lcd.setCursor(0, 0);
  lcd.print("SP:");

  lcd.setCursor(13, 0);
  lcd.print("mm ");

  bool blankSP = (enteringValue && currentParam == SETPOINT && !blinkOn);

  if (blankSP) {
    lcd.setCursor(3,0);
    lcd.print("       ");
  } else {
    float val = ref_mm;
    if (enteringValue && currentParam == SETPOINT) val = currentValue;
    lcdPrintFloat(val, 3, 0, 2);
  }

  // ------------- Satır 1: Kp ve Ki -------------
  lcd.setCursor(0,1); lcd.print("Kp:");
  lcd.setCursor(10,1); lcd.print("Ki:");

  bool blankKp = (enteringValue && currentParam == KP_ && !blinkOn);
  bool blankKi = (enteringValue && currentParam == KI_ && !blinkOn);

  if (blankKp) {
    lcd.setCursor(3,1); lcd.print("       ");
  } else {
    float val = Kp[0];
    if (enteringValue && currentParam == KP_) val = currentValue;
    lcdPrintFloat(val, 3, 1, 2);
  }

  if (blankKi) {
    lcd.setCursor(13,1); lcd.print("       ");
  } else {
    float val = Ki[0];
    if (enteringValue && currentParam == KI_) val = currentValue;
    lcdPrintFloat(val, 13, 1, 2);
  }

  // ------------- Satır 2: Kd ve Mode -------------
  lcd.setCursor(0,2); lcd.print("Kd:");
  bool blankKd = (enteringValue && currentParam == KD_ && !blinkOn);

  if (blankKd) {
    lcd.setCursor(3,2); lcd.print("       ");
  } else {
    float val = Kd[0];
    if (enteringValue && currentParam == KD_) val = currentValue;
    lcdPrintFloat(val, 3, 2, 2);
  }

  lcd.setCursor(10,2); lcd.print("Mode:");
  lcd.setCursor(15,2);
  switch (mode) {
    case MODE_P:   lcd.print("P  ");   break;
    case MODE_PI:  lcd.print("PI ");   break;
    case MODE_PD:  lcd.print("PD ");   break;
    case MODE_PID: lcd.print("PID");   break;
  }

  // ------------- Satır 3: Ölçülen Z -------------
  lcd.setCursor(0,3);
  lcd.print("Z:");
  lcdPrintFloat(lastZ, 3, 3, 2);
  lcd.setCursor(10,3);
  lcd.print("mm     ");
}


// ---------------------------------------------------
// Keypad Menü Kontrolü
// ---------------------------------------------------
void handleMenu() {

  char key = keypad.getKey();
  if (key == NO_KEY) return;

  // ---------------- Sayısal tuşlar ----------------
  if (key >= '0' && key <= '9') {
    if (enteringValue) {
      int digit = key - '0';
      if (!hasDecimal) {
        currentValue = currentValue * 10.0 + digit;
      } else {
        decimalFactor *= 0.1;            // 0.1 -> 0.01 -> ...
        currentValue += digit * decimalFactor;
      }
      Serial.print(F("Input = "));
      Serial.println(currentValue, 4);
      updateLCD();
    }
    return;
  }

  // ---------------- Değer girme modundaysak özel tuşlar ----------------
  if (enteringValue) {

    if (key == 'C') {       // temizle
      clearInput();
      updateLCD();
      return;
    }

    if (key == 'D') {       // en sag alt tus -> onay
      applyValue();
      updateLCD();
      return;
    }

    if (key == '*') {       // ondalik nokta
      if (!hasDecimal) {
        hasDecimal    = true;
        decimalFactor = 1.0;
        Serial.println(F("Ondalik mod: * basildi (.)"));
      }
      return;
    }

    // deger girilirken diğer tuşları görmezden gel
    return;
  }

  // ---------------- (buradan sonrası enteringValue = false iken) ----------------

  if (key == '*') {          // menüyü göster
    showMenuSerial();
    return;
  }

  // ----- MOD SEÇİMİ -----
  if (key == 'A') {
    mode = MODE_P;
    Serial.println(F("Mode = P"));
    updateLCD();
    return;
  }
  if (key == 'B') {
    mode = MODE_PI;
    Serial.println(F("Mode = PI"));
    updateLCD();
    return;
  }
  if (key == 'C') {          // PD modu
    mode = MODE_PD;
    Serial.println(F("Mode = PD"));
    updateLCD();
    return;
  }
  if (key == '#') {
    mode = MODE_PID;
    Serial.println(F("Mode = PID"));
    updateLCD();
    return;
  }

  // ----- PARAMETRE SEÇİMİ (1–4) -----
  if (key == '1') {          // Setpoint
    currentParam  = SETPOINT;
    enteringValue = true;
    clearInput();
    Serial.println(F("Setpoint gir (mm):"));
    return;
  }

  if (key == '2') {          // Kp
    currentParam  = KP_;
    enteringValue = true;
    clearInput();
    Serial.println(F("Kp gir:"));
    return;
  }

  if (key == '3') {          // Ki
    currentParam  = KI_;
    enteringValue = true;
    clearInput();
    Serial.println(F("Ki gir:"));
    return;
  }

  if (key == '4') {          // Kd
    currentParam  = KD_;
    enteringValue = true;
    clearInput();
    Serial.println(F("Kd gir:"));
    return;
  }

  // D'ye basılırsa ama şu an değer girilmiyorsa:
  if (key == 'D') {
    Serial.println(F("Onay icin once 1–4 ile bir param secip deger girmeniz gerekir."));
  }
}


// ---------------------------------------------------
// VL53 Ölçüm
// ---------------------------------------------------
float readToF() {
  int mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) return -1;
  return (float)mm;
}


// ---------------------------------------------------
// PID Hesabı
// ---------------------------------------------------
float computePID(int id, float ref, float meas) {
  float error = ref - meas;

  integral[id]   += error * Ts;
  derivative[id]  = (error - prevError[id]) / Ts;

  float u = 0.0;

  switch (mode) {
    case MODE_P:
      u = Kp[id] * error;
      break;

    case MODE_PI:
      u = Kp[id] * error + Ki[id] * integral[id];
      break;

    case MODE_PD:
      u = Kp[id] * error + Kd[id] * derivative[id];
      break;

    case MODE_PID:
      u = Kp[id] * error + Ki[id] * integral[id] + Kd[id] * derivative[id];
      break;
  }

  prevError[id] = error;

  if (u >  255.0) u =  255.0;
  if (u < -255.0) u = -255.0;

  return u;
}


// ---------------------------------------------------
// Bobin Sürme
// ---------------------------------------------------
void driveCoil(int EN, int IN1, int IN2, float u) {

  if (u >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN, (int)u);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN, (int)(-u));
  }
}


// ---------------------------------------------------
// SETUP
// ---------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(EN1, OUTPUT); pinMode(IN1_1, OUTPUT); pinMode(IN1_2, OUTPUT);
  pinMode(EN2, OUTPUT); pinMode(IN2_1, OUTPUT); pinMode(IN2_2, OUTPUT);
  pinMode(EN3, OUTPUT); pinMode(IN3_1, OUTPUT); pinMode(IN3_2, OUTPUT);
  pinMode(EN4, OUTPUT); pinMode(IN4_1, OUTPUT); pinMode(IN4_2, OUTPUT);

  initToF();

  lcd.begin(20, 4);
  lcd.clear();

  lastTime = millis();

  Serial.println(F("Sistem Basladi."));
  showMenuSerial();
  updateLCD();
}


// ---------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------
void loop() {

  handleMenu();   // keypad menü kontrol
  updateLCD();    // ekrani sürekli güncel tut

  if (millis() - lastTime >= Ts * 1000) {
    lastTime = millis();

    lastZ = readToF();   // mm

    // 4 bobin PID çıkışları
    float u1 = computePID(0, ref_mm, lastZ);
    float u2 = computePID(1, ref_mm, lastZ);
    float u3 = computePID(2, ref_mm, lastZ);
    float u4 = computePID(3, ref_mm, lastZ);

    // bobinleri sür
    driveCoil(EN1, IN1_1, IN1_2, u1);
    driveCoil(EN2, IN2_1, IN2_2, u2);
    driveCoil(EN3, IN3_1, IN3_2, u3);
    driveCoil(EN4, IN4_1, IN4_2, u4);

    // Debug
    Serial.print(F("Z = "));  Serial.print(lastZ);
    Serial.print(F(" mm | SP = ")); Serial.print(ref_mm, 3);
    Serial.print(F(" | u1=")); Serial.print(u1);
    Serial.print(F(" u2="));  Serial.print(u2);
    Serial.print(F(" u3="));  Serial.print(u3);
    Serial.print(F(" u4="));  Serial.println(u4);
  }
}
