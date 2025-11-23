#include "lcd_ui.h"
#include <LiquidCrystal.h>

// ---------------- LCD Pinleri ----------------
// RS=16, EN=17, D4=23, D5=25, D6=27, D7=29
LiquidCrystal lcd(16, 17, 23, 25, 27, 29);

// ---------------- Encoder + Buton Pinleri ----------------
const int ENC_A = 31;
const int ENC_B = 33;
const int BTN   = 35;

// ---------------- Debounce ----------------
static unsigned long lastButtonTime = 0;
const unsigned long debounceDelay = 120;  // ms

// ---------------- PID Parametreleri (UI tarafı) ----------------
// Bunlar hem kullanıcı girişini, hem de (autotune ON iken)
// controller'dan gelen anlık kazançları göstermek için kullanılıyor.
static float ref_mm = 20.0f;   // 10–40 mm arası, başlangıç 20
static float Kp_val = 1.50f;
static float Ki_val = 0.00f;
static float Kd_val = 0.00f;

static ControlMode mode = MODE_PID;
static bool autoTuneOn = false;

// ---------------- UI Durumu ----------------
// 0: SP, 1: Kp, 2: Ki, 3: Kd, 4: Mode, 5: AutoTune
static int  selectedIndex = 0;
static bool editMode      = false;
static int  lastEncA      = 0;


// --------------------------------------------------------
// Sensör mesafesini LCD'de göster (satır 3, kolon 8'den itibaren)
// --------------------------------------------------------
void lcdSetDistance(int mm)
{
    lcd.setCursor(8, 3);
    lcd.print("Z:");
    if (mm < 0) {
        lcd.print("----");
    } else {
        lcd.print(mm);
        lcd.print("mm");
    }
    lcd.print("  "); // eski karakterleri temizlemek için
}


// --------------------------------------------------------
// Yardımcı: Seçilen parametreyi encoder yönüne göre ayarla
// direction: +1 veya -1
// --------------------------------------------------------
static void adjustValue(int direction)
{
    switch (selectedIndex)
    {
        case 0: // SP
            ref_mm += direction;
            if (ref_mm < 10.0f) ref_mm = 10.0f;
            if (ref_mm > 40.0f) ref_mm = 40.0f;
            break;

        case 1: // Kp
            Kp_val += direction * 0.005f;
            if (Kp_val < 0.0f) Kp_val = 0.0f;
            break;

        case 2: // Ki
            Ki_val += direction * 0.005f;
            if (Ki_val < 0.0f) Ki_val = 0.0f;
            break;

        case 3: // Kd
            Kd_val += direction * 0.005f;
            if (Kd_val < 0.0f) Kd_val = 0.0f;
            break;

        case 4: // Mode (P, PI, PD, PID)
        {
            int m = (int)mode;
            m += direction;
            if (m < (int)MODE_P)   m = (int)MODE_PID;
            if (m > (int)MODE_PID) m = (int)MODE_P;
            mode = (ControlMode)m;
            break;
        }

        case 5: // AutoTune ON/OFF
            if (direction != 0) {
                autoTuneOn = !autoTuneOn;  // her tıkta toggle
            }
            break;
    }
}


// --------------------------------------------------------
// Encoder okuma
// --------------------------------------------------------
static void readEncoder()
{
    int A = digitalRead(ENC_A);

    if (A != lastEncA) {
        bool cw = (digitalRead(ENC_B) != A);  // Clockwise?
        int dir = cw ? +1 : -1;

        if (editMode) {
            // Değer düzenleme modunda: parametreyi değiştir
            adjustValue(dir);
        } else {
            // Menü modunda: hangi parametre seçili?
            selectedIndex += dir;
            if (selectedIndex < 0) selectedIndex = 5;
            if (selectedIndex > 5) selectedIndex = 0;
        }
    }

    lastEncA = A;
}


// --------------------------------------------------------
// Buton okuma (editMode toggle + debounce)
// --------------------------------------------------------
static void readButton()
{
    if (digitalRead(BTN) == LOW) {
        unsigned long now = millis();
        if (now - lastButtonTime > debounceDelay) {
            editMode = !editMode;   // edit <-> menu geçiş
            lastButtonTime = now;
        }
    }
}


// --------------------------------------------------------
// LCD güncelleme (SP, Kp, Ki, Kd, Mode, AutoTune)
// Z satırının sağ tarafına lcdSetDistance() dokunuyor
// --------------------------------------------------------
static void updateLCD()
{
    // --- Satır 0: SP ve Mode ---
    // SP
    lcd.setCursor(0, 0);
    lcd.print((selectedIndex == 0) ? ">" : " ");
    lcd.print("SP:");
    lcd.print(ref_mm, 0);
    lcd.print("mm  ");

    // Mode
    lcd.setCursor(10, 0);
    lcd.print((selectedIndex == 4) ? ">" : " ");
    lcd.print("Md:");
    switch (mode) {
        case MODE_P:   lcd.print("P  ");   break;
        case MODE_PI:  lcd.print("PI ");   break;
        case MODE_PD:  lcd.print("PD ");   break;
        case MODE_PID: lcd.print("PID");   break;
    }

    // Edit mod göstergesi (sağ üst köşe)
    lcd.setCursor(19, 0);
    lcd.print(editMode ? "E" : " ");

    // --- Satır 1: Kp ---
    lcd.setCursor(0, 1);
    lcd.print((selectedIndex == 1) ? ">" : " ");
    lcd.print("Kp:");
    lcd.print(Kp_val, 3);
    lcd.print("   ");

    // --- Satır 2: Ki ve Kd ---
    // Ki
    lcd.setCursor(0, 2);
    lcd.print((selectedIndex == 2) ? ">" : " ");
    lcd.print("Ki:");
    lcd.print(Ki_val, 3);
    lcd.print("  ");

    // Kd
    lcd.setCursor(10, 2);
    lcd.print((selectedIndex == 3) ? ">" : " ");
    lcd.print("Kd:");
    lcd.print(Kd_val, 3);
    lcd.print("  ");

    // --- Satır 3: AutoTune + Z (Z sağ tarafta) ---
    lcd.setCursor(0, 3);
    lcd.print((selectedIndex == 5) ? ">" : " ");
    lcd.print("AT:");
    lcd.print(autoTuneOn ? "ON " : "OFF");
    // Z bilgisi lcdSetDistance() ile sağ tarafa yazılıyor
}


// --------------------------------------------------------
// PUBLIC: LCD + encoder UI başlatma
// --------------------------------------------------------
void lcdUIInit()
{
    lcd.begin(20, 4);
    lcd.clear();

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    pinMode(BTN,   INPUT_PULLUP);

    lastEncA = digitalRead(ENC_A);

    lcd.setCursor(0, 0);
    lcd.print("ELE301 PID Control");
    delay(600);
    lcd.clear();

    updateLCD();
}


// --------------------------------------------------------
// PUBLIC: Her loop turunda çağrılacak UI task'i
// --------------------------------------------------------
void lcdUITask()
{
    readEncoder();
    readButton();
    updateLCD();
}


// --------------------------------------------------------
// PUBLIC: Getter fonksiyonlar
// --------------------------------------------------------
float getRefMm()             { return ref_mm; }
float getKp()                { return Kp_val; }
float getKi()                { return Ki_val; }
float getKd()                { return Kd_val; }
ControlMode getControlMode() { return mode; }
bool getAutoTuneEnabled()    { return autoTuneOn; }
bool lcdIsEditing()          { return editMode; }


// --------------------------------------------------------
// PUBLIC: Autotune'dan gelen Kp/Ki/Kd'yi LCD'ye yansıt
//
// Autotune aktifken main, her PID adımında burayı çağırabilir:
//   lcdUpdateGainsFromController(Kp_active, Ki_active, Kd_active);
//
// Böylece ekrandaki Kp/Ki/Kd de kontrolcüyle aynı olur.
// Kullanıcı edit modundayken AUTOTUNE LCD'yi bozmasın diye
// editMode == false iken güncelliyoruz.
// --------------------------------------------------------
void lcdUpdateGainsFromController(float Kp, float Ki, float Kd)
{
    if (!editMode) {
        Kp_val = Kp;
        Ki_val = Ki;
        Kd_val = Kd;
    }
}
