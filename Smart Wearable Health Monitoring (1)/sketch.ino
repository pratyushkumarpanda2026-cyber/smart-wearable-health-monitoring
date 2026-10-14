#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---------- Pins (apni wiring se match karo) ----------
#define MQ2        34
#define PULSE_POT  32
#define BUZZER     13
#define LED        2
#define SOS        15
#define ONE_WIRE   4
#define MPU_ADDR   0x68

// ---------- Thresholds ----------
const int   GAS_WARN = 2000,  GAS_DANGER = 3000;
const float TEMP_WARN = 45.0, TEMP_DANGER = 60.0;
const int   HR_LOW = 50,      HR_HIGH = 150;
const float FALL_G = 2.5;                 // impact g
const unsigned long CANCEL_MS   = 5000;   // fall cancel time
const unsigned long NOMOTION_MS = 20000;  // man-down time

OneWire oneWire(ONE_WIRE);
DallasTemperature sensors(&oneWire);

enum State { NORMAL, WARNING, DANGER, SOS_ALERT };
State state = NORMAL;

int gasValue = 0, heartRate = 0;
float temperature = 0, accG = 1.0;
bool fallPending = false, fallSOS = false, manDown = false;
unsigned long fallTime = 0, lastMotion = 0;
unsigned long lastSensor = 0, lastPrint = 0, lastBlink = 0;
bool blinkOn = false;

float readAccelG() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  float x = ax / 16384.0, y = ay / 16384.0, z = az / 16384.0;
  return sqrt(x * x + y * y + z * z);
}

void setup() {
  Serial.begin(115200);
  pinMode(MQ2, INPUT);
  pinMode(PULSE_POT, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(SOS, INPUT_PULLUP);

  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);          // MPU6050 wake up
  Wire.endTransmission(true);

  sensors.begin();
  sensors.setWaitForConversion(false);      // non-blocking
  sensors.requestTemperatures();
  lastMotion = millis();

  Serial.println("Smart Safety Module Started");
}

void loop() {
  unsigned long now = millis();
  bool btn = (digitalRead(SOS) == LOW);

  // ---- MPU6050: fast check (fall + man-down) ----
  accG = readAccelG();
  if (fabs(accG - 1.0) > 0.15) { lastMotion = now; manDown = false; }
  if (now - lastMotion > NOMOTION_MS) manDown = true;

  if (accG > FALL_G && !fallPending && !fallSOS) {
    fallPending = true;
    fallTime = now;
    Serial.println("FALL DETECTED! Press SOS button within 5s to cancel");
  }
  if (fallPending) {
    if (btn) { fallPending = false; Serial.println("Fall cancelled"); }
    else if (now - fallTime > CANCEL_MS) {
      fallPending = false; fallSOS = true;
      Serial.println("AUTO SOS (fall, no response)");
    }
  }
  if (fallSOS && btn) fallSOS = false;      // button se reset

  // ---- Slow sensors: every 500 ms ----
  if (now - lastSensor >= 500) {
    lastSensor = now;
    gasValue = analogRead(MQ2);
    heartRate = map(analogRead(PULSE_POT), 0, 4095, 40, 200);
    float t = sensors.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C) temperature = t;
    else Serial.println("Temp sensor error!");
    sensors.requestTemperatures();
  }

  // ---- State machine ----
  bool manualSOS = btn && !fallPending;
  if (manualSOS || fallSOS || manDown)
    state = SOS_ALERT;
  else if (gasValue > GAS_DANGER || temperature > TEMP_DANGER)
    state = DANGER;
  else if (gasValue > GAS_WARN || temperature > TEMP_WARN ||
           heartRate < HR_LOW || heartRate > HR_HIGH)
    state = WARNING;
  else
    state = NORMAL;

  // ---- Buzzer / LED patterns ----
  unsigned long interval = (state == WARNING) ? 600 : (state == DANGER) ? 150 : 0;
  bool out = false;
  if (state == SOS_ALERT) out = true;                 // continuous
  else if (state == NORMAL) out = false;
  else {
    if (now - lastBlink >= interval) { lastBlink = now; blinkOn = !blinkOn; }
    out = blinkOn;
  }
  digitalWrite(BUZZER, out);
  digitalWrite(LED, out);

  // ---- Serial report: every 1 s ----
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    const char* names[] = {"NORMAL", "WARNING", "DANGER", "SOS"};
    Serial.printf("Gas:%d Temp:%.1fC HR:%d Acc:%.2fg State:%s%s\n",
                  gasValue, temperature, heartRate, accG, names[state],
                  manDown ? " [MAN-DOWN]" : "");
  }
}

