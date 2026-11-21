/*  quadbot_batt.h  --  battery voltage monitor for a 2S LiPo (QuadBot-4 robot).

    Wiring: a voltage divider from the battery (+), after the power switch, to A0.
        battery + --- 10 k --- A0 --- 4.7 k --- GND        (optional 0.1 uF A0-GND)
    10 k / 4.7 k scales 8.4 V down to 2.7 V and reads up to 15.6 V, so a 9 V
    battery on the same divider is safe too.  The divider only draws 0.6 mA,
    and nothing while the switch is off.

    Below 3.0 V on the divider the monitor decides nothing is wired and stays
    quiet, so the sketch runs as before without it.

    Levels (2S, per cell):  LOW below 3.45 V (6.9 V) for 5 s  -> warning;
    EMPTY below 3.30 V (6.6 V) for 5 s -> the robot sits down and will not walk
    until the battery reads above 3.60 V per cell again (a fresh battery).
    Readings are smoothed over about 2 s so a servo surge does not trigger it.

    Calibration: measure the battery with a multimeter and send "B 7.95" (your
    reading) over serial; the correction is saved in EEPROM bytes 16-18.        */
#pragma once
#include <Arduino.h>
#include <EEPROM.h>

const uint8_t BATT_PIN       = A0;
const uint8_t BATT_CELLS     = 2;
const float   BATT_R_TOP     = 10000.0f;
const float   BATT_R_BOTTOM  = 4700.0f;
const float   BATT_VREF      = 5.0f;        // Uno R4 analog reference (its 5 V rail)
const float   BATT_ABSENT_V  = 3.0f;        // below this: no divider wired
const float   CELL_LOW       = 3.45f;
const float   CELL_EMPTY     = 3.30f;
const float   CELL_RECOVER   = 3.60f;
const uint16_t BATT_HOLD_MS  = 5000;        // a level must hold this long to count

enum { BATT_NONE, BATT_OK, BATT_LOW, BATT_EMPTY };

const uint8_t EE_BATT_MAGIC = 0xBA;
float    battScale   = 1.0f;     // multimeter correction
float    battV       = 0;        // smoothed battery voltage
uint8_t  battState   = BATT_NONE;
uint32_t battLastRead = 0, battBelowSince = 0, battAboveSince = 0;

inline float battReadRaw() {                        // one averaged reading, in volts
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 16; i++) sum += analogRead(BATT_PIN);
  float a = sum / 16.0f / 16383.0f * BATT_VREF;     // 14-bit
  return a * (BATT_R_TOP + BATT_R_BOTTOM) / BATT_R_BOTTOM * battScale;
}

// Charge left, 0-100 %, from the resting voltage of one LiPo cell.  Under load
// the voltage reads lower, so the figure dips while walking.
inline uint8_t battPercent() {
  static const float V[] = {3.30f, 3.45f, 3.55f, 3.65f, 3.70f, 3.75f, 3.80f, 3.85f, 3.95f, 4.10f, 4.20f};
  static const uint8_t P[] = {0, 3, 8, 15, 25, 35, 45, 55, 75, 90, 100};
  float c = battV / BATT_CELLS;
  if (c <= V[0]) return 0;
  for (uint8_t i = 1; i < 11; i++)
    if (c < V[i]) return P[i - 1] + (P[i] - P[i - 1]) * (c - V[i - 1]) / (V[i] - V[i - 1]);
  return 100;
}

inline void battBegin() {
  analogReadResolution(14);
  if (EEPROM.read(16) == EE_BATT_MAGIC) {
    uint16_t s = EEPROM.read(17) | (EEPROM.read(18) << 8);
    if (s >= 8000 && s <= 12000) battScale = s / 10000.0f;
  }
  battV = battReadRaw();
  battState = battV < BATT_ABSENT_V ? BATT_NONE : BATT_OK;
  if (battState == BATT_OK && battV / BATT_CELLS < CELL_EMPTY) battState = BATT_EMPTY;  // flat at power-up
  else if (battState == BATT_OK && battV / BATT_CELLS < CELL_LOW) battState = BATT_LOW;
}

// Call often; it reads every 200 ms.  Returns true when the state changed.
inline bool battUpdate(uint32_t now) {
  if (now - battLastRead < 200) return false;
  battLastRead = now;
  float v = battReadRaw();
  uint8_t old = battState;
  if (v < BATT_ABSENT_V) {                          // unplugged or switched off (USB power)
    battV = v; battState = BATT_NONE; battBelowSince = battAboveSince = 0;
    return old != battState;
  }
  if (battState == BATT_NONE) { battV = v; battState = BATT_OK; }
  battV += (v - battV) * 0.1f;                      // about 2 s time constant
  float c = battV / BATT_CELLS;

  if (battState == BATT_EMPTY) {                    // latched until a charged battery
    if (c > CELL_RECOVER) { if (!battAboveSince) battAboveSince = now; }
    else battAboveSince = 0;
    if (battAboveSince && now - battAboveSince > 2000) { battState = BATT_OK; battAboveSince = 0; }
    return old != battState;
  }
  uint8_t want = c < CELL_EMPTY ? BATT_EMPTY : c < CELL_LOW ? BATT_LOW : BATT_OK;
  if (want > battState) {                           // getting worse: must hold BATT_HOLD_MS
    if (!battBelowSince) battBelowSince = now;
    if (now - battBelowSince > BATT_HOLD_MS) { battState = want; battBelowSince = 0; }
  } else {
    battBelowSince = 0;
    if (want == BATT_OK && battState == BATT_LOW && c > CELL_LOW + 0.05f) battState = BATT_OK;
  }
  return old != battState;
}

inline void battCalibrate(float measured) {
  float raw = battReadRaw() / battScale;
  if (raw < BATT_ABSENT_V || measured < BATT_ABSENT_V) { Serial.println(F("no battery reading to calibrate")); return; }
  float s = measured / raw;
  if (s < 0.8f || s > 1.2f) { Serial.println(F("more than 20 % off: check the divider resistors")); return; }
  battScale = s;
  uint16_t q = (uint16_t)(s * 10000 + 0.5f);
  EEPROM.update(16, EE_BATT_MAGIC);
  EEPROM.update(17, q & 0xFF);
  EEPROM.update(18, q >> 8);
  battV = battReadRaw();
  Serial.print(F("battery calibrated, correction x")); Serial.println(s, 4);
}

inline const __FlashStringHelper* battStateName() {
  switch (battState) {
    case BATT_OK:    return F("ok");
    case BATT_LOW:   return F("LOW");
    case BATT_EMPTY: return F("EMPTY");
    default:         return F("no sensor");
  }
}

inline void battPrint() {
  Serial.print(F("battery "));
  if (battState == BATT_NONE) { Serial.println(F("not connected (A0 divider)")); return; }
  Serial.print(battV, 2); Serial.print(F(" V  "));
  Serial.print(battPercent()); Serial.print(F(" %  "));
  Serial.println(battStateName());
}
