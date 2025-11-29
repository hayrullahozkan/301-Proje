#include "lcd_ui.h"
#include <LiquidCrystal.h>

// ---------------- LCD Pins ----------------
LiquidCrystal lcd(16, 17, 23, 25, 27, 29);

// ---------------- Encoder Pins ----------------
#define ENC_A   31
#define ENC_B   33
#define ENC_BTN 35

// Encoder state
static int lastA = HIGH;
static int lastB = HIGH;
static int encDelta = 0;
static unsigned long lastEncTime = 0;
const unsigned long ENC_DEBOUNCE = 2;

// Button state
static bool lastBtn = HIGH;
static unsigned long lastBtnTime = 0;

// ---------------- UI Values ----------------
static float ref_mm = 20.0;
static float Kp     = 1.500;
static float Ki     = 0.000;
static float Kd     = 0.000;

static ControlMode mode = MODE_PID;
static bool autoTuneEnabled = false;

static bool editing = false;
static int menuIndex = 0;

// ---------------- Raw Encoder ----------------
void readEncoderRaw()
{
    unsigned long now = millis();
    if (now - lastEncTime < ENC_DEBOUNCE) return;

    int A = digitalRead(ENC_A);
    int B = digitalRead(ENC_B);

    if (A != lastA)
    {
        if (A == LOW)
        {
            if (B == HIGH) encDelta++;
            else           encDelta--;
        }
        lastEncTime = now;
    }

    lastA = A;
    lastB = B;
}

int readEncoderDelta()
{
    int d = encDelta;
    encDelta = 0;
    return d;
}

// ---------------- Button Debounce ----------------
bool readButton()
{
    int s = digitalRead(ENC_BTN);
    unsigned long now = millis();

    if (s != lastBtn && (now - lastBtnTime) > 20)
    {
        lastBtn = s;
        lastBtnTime = now;
        if (s == LOW) return true;
    }
    return false;
}

// -------------------------------------------------
// LCD DRAW WITH FIXED-WIDTH FIELDS
// -------------------------------------------------
void printFixedFloat(int col, int row, float val, uint8_t prec, uint8_t width)
{
    lcd.setCursor(col, row);

    // Clear area
    for (int i = 0; i < width; i++)
        lcd.print(" ");

    lcd.setCursor(col, row);
    lcd.print(val, prec);
}

void printFixedText(int col, int row, const char* txt, uint8_t width)
{
    lcd.setCursor(col, row);
    for (int i = 0; i < width; i++)
        lcd.print(" ");

    lcd.setCursor(col, row);
    lcd.print(txt);
}

// -------------------------------------------------
void drawScreen()
{
    lcd.clear();

    // -------- Row 0: SP + Distance --------
    lcd.setCursor(0,0);
    lcd.print((menuIndex == 0) ? (editing ? ">" : "*") : " ");
    lcd.print("SP:");

    printFixedFloat(3, 0, ref_mm, 1, 6);

    lcd.setCursor(12,0);
    lcd.print("Z:    ");

    // -------- Row 1: Kp + Mode --------
    lcd.setCursor(0,1);
    lcd.print((menuIndex == 1) ? (editing ? ">" : "*") : " ");
    lcd.print("Kp:");
    printFixedFloat(3, 1, Kp, 3, 6);

    lcd.setCursor(12,1);
    lcd.print((menuIndex == 4) ? (editing ? ">" : "*") : " ");
    lcd.print("M:");

    switch(mode){
        case MODE_P:   printFixedText(15,1,"P  ",3); break;
        case MODE_PI:  printFixedText(15,1,"PI ",3); break;
        case MODE_PD:  printFixedText(15,1,"PD ",3); break;
        case MODE_PID: printFixedText(15,1,"PID",3); break;
    }

    // -------- Row 2: Ki + AT --------
    lcd.setCursor(0,2);
    lcd.print((menuIndex == 2) ? (editing ? ">" : "*") : " ");
    lcd.print("Ki:");
    printFixedFloat(3, 2, Ki, 3, 6);

    lcd.setCursor(12,2);
    lcd.print((menuIndex == 5) ? (editing ? ">" : "*") : " ");
    lcd.print("AT:");
    printFixedText(15, 2, autoTuneEnabled ? "ON " : "OFF", 3);

    // -------- Row 3: Kd --------
    lcd.setCursor(0,3);
    lcd.print((menuIndex == 3) ? (editing ? ">" : "*") : " ");
    lcd.print("Kd:");
    printFixedFloat(3, 3, Kd, 3, 6);
}

// -------------------------------------------------
void lcdSetDistance(int mm)
{
    lcd.setCursor(15,0);
    printFixedFloat(15, 0, (float)mm, 0, 4);
}

// -------------------------------------------------
float getRefMm() { return ref_mm; }
float getKp()    { return Kp; }
float getKi()    { return Ki; }
float getKd()    { return Kd; }
ControlMode getControlMode() { return mode; }
bool getAutoTuneEnabled() { return autoTuneEnabled; }
bool lcdIsEditing() { return editing; }

void lcdUpdateGainsFromController(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    drawScreen();
}

// -------------------------------------------------
void lcdUIInit()
{
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    pinMode(ENC_BTN, INPUT_PULLUP);

    lcd.begin(20,4);
    drawScreen();
}

// -------------------------------------------------
void lcdUITask()
{
    readEncoderRaw();
    int d = readEncoderDelta();

    if (d != 0)
    {
        if (!editing)
        {
            menuIndex += (d > 0 ? 1 : -1);
            if (menuIndex < 0) menuIndex = 5;
            if (menuIndex > 5) menuIndex = 0;
        }
        else
        {
            switch(menuIndex){
                case 0:  // SP
                    ref_mm += (d > 0 ? 1 : -1);
                    if (ref_mm < 5) ref_mm = 5;
                    if (ref_mm > 200) ref_mm = 200;
                    break;

                case 1:  // Kp
                    Kp += (d > 0 ? 0.005 : -0.005);
                    if (Kp < 0.0) Kp = 0.0;
                    if (Kp > 9.999) Kp = 9.999;
                    break;

                case 2:  // Ki
                    Ki += (d > 0 ? 0.005 : -0.005);
                    if (Ki < 0.0) Ki = 0.0;
                    if (Ki > 100.0) Ki = 100.0;
                    break;

                case 3:  // Kd
                    Kd += (d > 0 ? 0.005 : -0.005);
                    if (Kd < 0.0) Kd = 0.0;
                    if (Kd > 9.999) Kd = 9.999;
                    break;

                case 4:  // Mode
                    mode = (ControlMode)((int)mode + (d > 0 ? 1 : -1));
                    if (mode < 0) mode = MODE_PID;
                    if (mode > MODE_PID) mode = MODE_P;
                    break;

                case 5:  // AutoTune
                    autoTuneEnabled = !autoTuneEnabled;
                    break;
            }
        }

        drawScreen();
    }

    if (readButton())
    {
        editing = !editing;
        drawScreen();
    }
}
