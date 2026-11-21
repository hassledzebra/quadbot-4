/*  quadbot_stick.h  --  joystick handling shared by every QuadBot-4 sketch.

    Axis convention used everywhere:  x > 0 = stick to the RIGHT,
                                      y > 0 = stick pushed FORWARD (away from you),
    both scaled to -100 .. +100.                                                    */
#pragma once
#include <Arduino.h>

enum { B_FWD = 1, B_BACK = 2, B_LEFT = 4, B_RIGHT = 8, B_MODE = 16 };

const int8_t STICK_ON  = 35;   // % deflection that starts a direction
const int8_t STICK_OFF = 25;   // % it must fall below to end it (hysteresis)
const int8_t STICK_DEAD = 6;   // centre dead band, %

// Raw ADC reading (0..1023) -> -100..+100 around a measured centre.
inline int8_t stickScale(int raw, int centre) {
  long d = (long)raw - centre;
  long span = d >= 0 ? (long)(1023 - centre) : (long)centre;
  if (span < 1) span = 1;
  long v = d * 100L / span;
  if (v > -STICK_DEAD && v < STICK_DEAD) v = 0;
  if (v > 100) v = 100;
  if (v < -100) v = -100;
  return (int8_t)v;
}

// Stick position -> virtual button mask.  A diagonal sets two bits, which the
// gait engine turns into a curve.  The stick's push button acts as MODE.
inline uint8_t stickMask(int8_t x, int8_t y, bool pressed) {
  static uint8_t held = 0;
  uint8_t m = 0;
  bool f = (held & B_FWD)   ? (y >  STICK_OFF) : (y >  STICK_ON);
  bool b = (held & B_BACK)  ? (y < -STICK_OFF) : (y < -STICK_ON);
  bool r = (held & B_RIGHT) ? (x >  STICK_OFF) : (x >  STICK_ON);
  bool l = (held & B_LEFT)  ? (x < -STICK_OFF) : (x < -STICK_ON);
  if (f) m |= B_FWD;
  if (b) m |= B_BACK;
  if (r) m |= B_RIGHT;
  if (l) m |= B_LEFT;
  held = m;
  return m | (pressed ? B_MODE : 0);
}

// How far the stick is pushed picks the walking speed: 0, 1 or 2 (speed 1..3).
inline uint8_t stickSpeed(int8_t x, int8_t y) {
  int ax = x < 0 ? -x : x, ay = y < 0 ? -y : y;
  int m = ax > ay ? ax : ay;
  return m < 62 ? 0 : (m < 90 ? 1 : 2);
}

// ---- stick orientation (learned once, kept in EEPROM bytes 32-33) -------------
#include <EEPROM.h>
struct StickMap { bool swap; int8_t sx, sy; };
const int STICK_EE = 32;

inline StickMap stickMapLoad() {
  StickMap m = {false, 1, -1};               // guess until the stick is set up
  if (EEPROM.read(STICK_EE) == 0x5A) {
    uint8_t f = EEPROM.read(STICK_EE + 1);
    m.swap = f & 1; m.sx = (f & 2) ? -1 : 1; m.sy = (f & 4) ? -1 : 1;
  }
  return m;
}

inline void stickMapSave(const StickMap &m) {
  EEPROM.update(STICK_EE, 0x5A);
  EEPROM.update(STICK_EE + 1, (m.swap ? 1 : 0) | (m.sx < 0 ? 2 : 0) | (m.sy < 0 ? 4 : 0));
}

// a, b = scaled readings of the VRx and VRy pins -> robot x (right +), y (forward +)
inline void stickApply(const StickMap &m, int8_t a, int8_t b, int8_t &x, int8_t &y) {
  x = (int8_t)((m.swap ? b : a) * m.sx);
  y = (int8_t)((m.swap ? a : b) * m.sy);
}

// Guided set-up: push the stick FORWARD, let go, push it RIGHT, let go.
// feedback(step) is called continuously: 0 = waiting for forward,
// 1 = waiting for right, 2 = finished.
template <typename F>
StickMap stickLearn(uint8_t pinX, uint8_t pinY, int cx, int cy, F feedback) {
  StickMap m = {false, 1, 1};
  uint8_t step = 0;
  bool centred = false;
  while (step < 2) {
    int dx = analogRead(pinX) - cx, dy = analogRead(pinY) - cy;
    int ax = dx < 0 ? -dx : dx, ay = dy < 0 ? -dy : dy;
    if (ax < 100 && ay < 100) centred = true;
    if (centred && step == 0 && (ax > 300 || ay > 300)) {          // FORWARD
      m.swap = ay < ax;                                            // forward on VRx: swap
      m.sy = (m.swap ? dx : dy) > 0 ? 1 : -1;
      step = 1; centred = false;
    } else if (centred && step == 1) {                            // RIGHT, on the other pin
      int d = m.swap ? dy : dx, ad = d < 0 ? -d : d;
      if (ad > 300) { m.sx = d > 0 ? 1 : -1; step = 2; }
    }
    feedback(step);
    delay(10);
  }
  feedback(2);
  stickMapSave(m);
  return m;
}
