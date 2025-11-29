from reportlab.platypus import SimpleDocTemplate, Paragraph
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib.pagesizes import letter

doc_path = "Project_Connections_Guide.pdf"
styles = getSampleStyleSheet()
story = []

text = """
PROJECT CONNECTIONS GUIDE (ENGLISH)

1. Arduino Mega 2560
- Main MCU board powering coils, LCD, sensor, and encoder.

2. Power System
- 12V → L298N VS input.
- 5V → Arduino + LCD (common ground required).

3. L298N Motor Drivers
L298N #1:
 ENA→D2, IN1→D22, IN2→D26
 ENB→D4, IN3→D28, IN4→D30

L298N #2:
 ENA→D6, IN1→D24, IN2→D32
 ENB→D8, IN3→D34, IN4→D36

Coils:
 Coil1→OUT1/OUT2
 Coil2→OUT3/OUT4
 Coil3→OUT1/OUT2
 Coil4→OUT3/OUT4

4. VL53L0X Sensor
 VIN→5V
 GND→GND
 SDA→D20
 SCL→D21
 XSHUT→D7

5. LCD (RepRap 20x4)
EXP1:
 D35, D16(RS), D25, D29(EN), 5V
 D37, D17, D23, D27, GND

EXP2:
 D52, D53, D51, D41, NC
 D50, D31, D33, D49, GND

6. Rotary Encoder
 CLK→D40
 DT →D42
 SW →D44
 +5V, GND

7. Notes
- All grounds must be connected.
- Safe coil current: 0.12–0.22A.
- Do not run electromagnets without PWM.
"""

story.append(Paragraph(text.replace("\n", "<br/>"), styles['Normal']))

doc = SimpleDocTemplate(doc_path, pagesize=letter)
doc.build(story)

print("PDF created:", doc_path)
