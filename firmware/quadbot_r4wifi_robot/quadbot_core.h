/*  quadbot_core.h  --  QuadBot-4 gait engine, servos, calibration and storage.
    The same file sits in every robot sketch folder; the sketch supplies the
    input (wired joystick or Wi-Fi) through the four functions declared below.

    Gait: one servo per leg swings it fore/aft.  A leg is longest when vertical,
    so the diagonal pair nearer vertical carries the body and the other pair is
    lifted and can step.  Eight segments per cycle, one pair moving at a time;
    all legs are vertical at segments 0 and 4, which is where the robot starts,
    stops and changes direction.

      seg  pair  moves to   role
       0    A     +1        swing forward (B is vertical and carries the body)
       1    B     -1        stance: B pushes back, body advances
       2    A      0        stance: A pushes back, body advances
       3    B      0        swing back to vertical (A carries)
       4..7 the same with A and B exchanged                                     */
#pragma once
#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>
#include "quadbot_stick.h"

// ---- supplied by the sketch ------------------------------------------------
uint8_t inputMask(uint32_t now);  // B_FWD | B_BACK | B_LEFT | B_RIGHT | B_MODE, right now
uint8_t inputSpeed();             // 0..2 from stick deflection, or 255 = use stored speed
void    inputSerial(char c);      // sketch-specific serial commands
void    inputStatus(uint8_t state, uint8_t cmd, uint8_t speed);   // display hook

// ---- wiring ------------------------------------------------------------------
// Servo pins in the wiring diagram's order: D8, D9, D10, D11 sit at the corners
// that were FL, FR, RL, RR when the board's USB end was the front.
const uint8_t SERVO_PIN[4] = {8, 9, 10, 11};
const uint8_t LED_PIN      = LED_BUILTIN;

// Body turned round (the USB end is now the REAR).  The firmware then calls the
// old rear corners the front: FL = D11, FR = D10, RL = D9, RR = D8, and "foot
// forward" points the other way.  Default for a robot that has no saved
// setting; the serial 'F' command (and the phone page) switches it and saves it.
const bool BODY_REVERSED_DEFAULT = true;

// ---- tuning ------------------------------------------------------------------
const uint32_t AUTO_REST_MS    = 60000;   // sit down after this long idle (0 = never)
const float    US_PER_DEG      = 10.3f;   // servo pulse change per degree
const int      CENTER_US       = 1500;    // pulse for "leg vertical"
const float    STRIDE_DEFAULT  = 25.0f;   // leg swing each side of vertical (deg)
const float    STRIDE_MIN      = 12.0f;
const float    STRIDE_MAX      = 35.0f;   // legs on one side meet beyond ~40
const float    REST_DEG        = 55.0f;   // sit pose: front legs fwd, rear legs back
const float    LEG_LIMIT_DEG   = 65.0f;   // mechanical limit of the printed parts
const uint16_t HOLD_MS         = 1000;    // MODE hold: sit / save calibration
const uint16_t CAL_HOLD_MS     = 3000;    // MODE hold while sitting: calibration

// Time per gait segment for speed 1, 2, 3.  Swings move an unloaded pair.
const uint16_t T_SWING[3]  = {260, 170, 120};
const uint16_t T_STANCE[3] = {420, 260, 170};

// Per servo (D8..D11): +1 if a longer pulse swings the foot toward the USB end
// of the board.  Left and right servos are mirror images.  Calibration can flip
// any of these; the result is saved.  Direction and trim belong to the SERVO,
// so a calibration made before the body was turned round stays valid.
const int8_t DIR_DEFAULT[4] = {-1, +1, -1, +1};

// ---- types ---------------------------------------------------------------------
enum { FL, FR, RL, RR };
enum { CMD_STOP, CMD_FWD, CMD_BACK, CMD_SPIN_L, CMD_SPIN_R,
       CMD_ARC_FL, CMD_ARC_FR, CMD_ARC_BL, CMD_ARC_BR };
enum { ST_STAND, ST_WALK, ST_SIT, ST_REST, ST_WAKE, ST_CALIB };

const uint8_t LEG_GROUP[4] = {0, 1, 1, 0};   // pair A = FL + RR, pair B = FR + RL
const int8_t MIX[5][4] = {
  { 0,  0,  0,  0},     // stop
  {+1, +1, +1, +1},     // forward
  {-1, -1, -1, -1},     // back
  {-1, +1, -1, +1},     // spin left  (left side walks back, right side forward)
  {+1, -1, +1, -1},     // spin right
};
const uint8_t ARC_PAIR[4][2] = {   // a curve alternates a half-cycle of walk and of spin
  {CMD_FWD,  CMD_SPIN_L}, {CMD_FWD,  CMD_SPIN_R},
  {CMD_BACK, CMD_SPIN_R}, {CMD_BACK, CMD_SPIN_L},   // reversing steers like a car
};
struct Seg { uint8_t grp; int8_t to; uint8_t stance; };
const Seg SEGS[8] = {
  {0, +1, 0}, {1, -1, 1}, {0, 0, 1}, {1, 0, 0},
  {1, +1, 0}, {0, -1, 1}, {1, 0, 1}, {0, 0, 0},
};

// ---- state ---------------------------------------------------------------------
Servo   servo[4];                   // indexed by servo (D8..D11), not by leg
bool    attached      = false;
int8_t  servoDir[4];                // per servo, see DIR_DEFAULT
int8_t  trimDeg[4];                 // per servo: neutral trim, in servo degrees
bool    bodyReversed  = BODY_REVERSED_DEFAULT;

// leg (FL, FR, RL, RR as the robot walks now) -> servo index 0..3 (D8..D11)
inline uint8_t legServo(uint8_t leg) { return bodyReversed ? 3 - leg : leg; }
// +1 if a longer pulse on this leg's servo moves its foot FORWARD (as it walks now)
inline int8_t  legDir(uint8_t leg)   { return bodyReversed ? -servoDir[legServo(leg)] : servoDir[legServo(leg)]; }
float   strideDeg     = STRIDE_DEFAULT;
uint8_t speedLevel    = 1;          // stored speed 0..2, used when the input has no speed

float    legNow[4]    = {0, 0, 0, 0};   // current leg angles, + = foot forward
float    legFrom[4], legTo[4];
uint32_t moveT0       = 0;
uint16_t moveDur      = 1;
bool     moving       = false;

uint8_t  state        = ST_STAND;
uint8_t  segIdx       = 0;
int8_t   phase[2]     = {0, 0};
int8_t   mixNow[4]    = {0, 0, 0, 0};
uint8_t  activeCmd    = CMD_STOP;
uint8_t  arcToggle    = 0;
uint8_t  walkSpeed    = 1;
bool     hurry        = false;
uint8_t  seqStep      = 0;
uint32_t seqT0        = 0;
uint32_t idleSince    = 0;

uint8_t  btn = 0, btnRaw = 0;       // debounced / raw input masks
uint32_t btnRawSince  = 0, btnSince = 0;
uint8_t  btnCmd       = CMD_STOP;
uint8_t  serialCmd    = CMD_STOP;
bool     modeDown     = false, modeFired = false, waitRelease = false;
uint32_t modeT0       = 0;
bool     wakeRequest  = false, sitPending = false;

uint8_t  calLeg       = 0;
uint8_t  blinkCount   = 0;
uint32_t blinkT0      = 0;

// ---- storage -------------------------------------------------------------------
const uint8_t EE_MAGIC = 0xB4;

void loadConfig() {
  if (EEPROM.read(0) == EE_MAGIC) {
    for (uint8_t i = 0; i < 4; i++) {
      servoDir[i] = (EEPROM.read(1 + i) == 1) ? 1 : -1;
      int t = (int8_t)EEPROM.read(5 + i);
      trimDeg[i]  = (int8_t)constrain(t, -20, 20);
    }
    uint8_t sp = EEPROM.read(9);
    speedLevel = sp > 2 ? 2 : sp;
    float st = EEPROM.read(10);
    strideDeg  = constrain(st, STRIDE_MIN, STRIDE_MAX);
  } else {
    for (uint8_t i = 0; i < 4; i++) { servoDir[i] = DIR_DEFAULT[i]; trimDeg[i] = 0; }
  }
}

// Body orientation lives in its own bytes (12-13) so robots with an older saved
// calibration still pick up BODY_REVERSED_DEFAULT.
const uint8_t EE_ORIENT_MAGIC = 0xA7;
void loadOrientation() {
  bodyReversed = (EEPROM.read(12) == EE_ORIENT_MAGIC) ? EEPROM.read(13) == 1 : BODY_REVERSED_DEFAULT;
}
void saveOrientation() {
  EEPROM.update(12, EE_ORIENT_MAGIC);
  EEPROM.update(13, bodyReversed ? 1 : 0);
}

void saveConfig() {
  EEPROM.update(0, EE_MAGIC);
  for (uint8_t i = 0; i < 4; i++) {
    EEPROM.update(1 + i, servoDir[i] > 0 ? 1 : 0);
    EEPROM.update(5 + i, (uint8_t)trimDeg[i]);
  }
  EEPROM.update(9, speedLevel);
  EEPROM.update(10, (uint8_t)(strideDeg + 0.5f));
}

// ---- servos ----------------------------------------------------------------------
void writeLeg(uint8_t i, float deg) {
  deg = constrain(deg, -LEG_LIMIT_DEG, LEG_LIMIT_DEG);
  legNow[i] = deg;
  if (!attached) return;
  uint8_t s = legServo(i);
  float us = CENTER_US + (trimDeg[s] + legDir(i) * deg) * US_PER_DEG;
  servo[s].writeMicroseconds((int)(constrain(us, 600.0f, 2400.0f) + 0.5f));
}

void attachAll() {
  if (attached) return;
  attached = true;
  for (uint8_t i = 0; i < 4; i++) {        // one at a time: gentler on the supply
    writeLeg(i, legNow[i]);                // AVR keeps this pulse for the first frame
    servo[legServo(i)].attach(SERVO_PIN[legServo(i)]);
    writeLeg(i, legNow[i]);                // R4 resets to 1500 us on attach
    delay(120);
  }
}

// Swap which end is the front.  Only while standing (all legs vertical) or
// resting (servos off); the sit pose is the same seen from either end.
bool setBodyReversed(bool rev) {
  if (state != ST_STAND && state != ST_REST) return false;
  if (rev == bodyReversed) return true;
  bodyReversed = rev;
  for (uint8_t i = 0; i < 4; i++) legNow[i] = -legNow[3 - i];  // same physical pose
  if (attached) for (uint8_t i = 0; i < 4; i++) writeLeg(i, legNow[i]);
  saveOrientation();
  return true;
}

void detachAll() {
  for (uint8_t i = 0; i < 4; i++) servo[i].detach();
  attached = false;
}

void startMove(const float tgt[4], uint16_t ms) {
  for (uint8_t i = 0; i < 4; i++) { legFrom[i] = legNow[i]; legTo[i] = tgt[i]; }
  moveT0  = millis();
  moveDur = ms ? ms : 1;
  moving  = true;
}

bool updateMove() {                          // true once the current move is finished
  if (!moving) return true;
  float u = (float)(millis() - moveT0) / (float)moveDur;
  if (u >= 1.0f) { u = 1.0f; moving = false; }
  float e = u * u * (3.0f - 2.0f * u);       // ease in / ease out
  for (uint8_t i = 0; i < 4; i++) writeLeg(i, legFrom[i] + (legTo[i] - legFrom[i]) * e);
  return !moving;
}

void moveBlocking(const float tgt[4], uint16_t ms) {
  startMove(tgt, ms);
  while (!updateMove()) delay(5);
}

void startPairMove(bool front, float deg, uint16_t ms) {
  float t[4];
  for (uint8_t i = 0; i < 4; i++) t[i] = legNow[i];
  if (front) { t[FL] = deg; t[FR] = deg; } else { t[RL] = deg; t[RR] = deg; }
  startMove(t, ms);
}

// ---- LED -------------------------------------------------------------------------
void blink(uint8_t n) { blinkCount = n; blinkT0 = millis(); }

void updateLed(uint32_t now) {
  bool on;
  if (blinkCount) {
    uint32_t t = now - blinkT0;
    if (t < 300) on = false;
    else {
      t -= 300;
      if (t >= (uint32_t)blinkCount * 260) { blinkCount = 0; on = false; }
      else on = (t % 260) < 130;
    }
  } else switch (state) {
    case ST_WALK:  on = (now % 240) < 120; break;
    case ST_REST:  on = (now % 2000) < 40; break;
    case ST_CALIB: { uint16_t t = now % 1000; on = t < 80 || (t > 160 && t < 240); } break;
    default:       on = true;
  }
  digitalWrite(LED_PIN, on);
}

// ---- gait ------------------------------------------------------------------------
uint8_t currentSpeed() {
  uint8_t s = inputSpeed();
  if (serialCmd != CMD_STOP || s > 2) return speedLevel;
  return s;
}

void startSegment() {
  const Seg &s = SEGS[segIdx];
  phase[s.grp] = s.to;
  float t[4];
  for (uint8_t i = 0; i < 4; i++) t[i] = mixNow[i] * strideDeg * phase[LEG_GROUP[i]];
  uint8_t sp = hurry ? 2 : walkSpeed;
  startMove(t, s.stance ? T_STANCE[sp] : T_SWING[sp]);
}

// Only called while every leg is vertical.  Returns false to stay standing.
bool latchCommand(uint8_t cmd) {
  hurry = false;
  activeCmd = cmd;
  if (cmd == CMD_STOP) { arcToggle = 0; return false; }
  uint8_t basic = cmd;
  if (cmd >= CMD_ARC_FL) { basic = ARC_PAIR[cmd - CMD_ARC_FL][arcToggle]; arcToggle ^= 1; }
  else arcToggle = 0;
  for (uint8_t i = 0; i < 4; i++) mixNow[i] = MIX[basic][i];
  // Spins turn further per step when the feet step in toward the body centre;
  // all legs are vertical now, so pick the half-cycle that does that.
  if (basic == CMD_SPIN_L) segIdx = 0;
  if (basic == CMD_SPIN_R) segIdx = 4;
  return true;
}

void standStill(uint32_t now) { state = ST_STAND; idleSince = now; }

// ---- sit / wake -----------------------------------------------------------------
void beginSit(uint32_t now)  { state = ST_SIT;  seqStep = 0; seqT0 = now; startPairMove(true, REST_DEG, 700); }
void beginWake(uint32_t now) { state = ST_WAKE; seqStep = 0; seqT0 = now; attachAll(); startPairMove(false, 0, 700); }

void runSit(uint32_t now) {
  if (!updateMove()) return;
  switch (seqStep) {
    case 0: startPairMove(false, -REST_DEG, 700); seqStep = 1; break;
    case 1: seqT0 = now; seqStep = 2; break;
    case 2: if (now - seqT0 > 400) { detachAll(); state = ST_REST; wakeRequest = false; } break;
  }
}

void runWake(uint32_t now) {
  if (!updateMove()) return;
  if (seqStep == 0) { startPairMove(true, 0, 700); seqStep = 1; }
  else { phase[0] = phase[1] = 0; segIdx &= 4; standStill(now); }
}

// ---- calibration ----------------------------------------------------------------
void calNod() {                       // nods toward what the firmware thinks is FRONT
  float t[4] = {0, 0, 0, 0};
  t[calLeg] = 12; moveBlocking(t, 160);
  t[calLeg] = 0;  moveBlocking(t, 160);
}

void calShowForward() {
  float t[4] = {0, 0, 0, 0};
  t[calLeg] = 20; moveBlocking(t, 300);
  delay(450);
  t[calLeg] = 0;  moveBlocking(t, 300);
}

void calReport() {
  uint8_t s = legServo(calLeg);
  Serial.print(F("cal leg "));  Serial.print(calLeg);
  Serial.print(F(" (D"));       Serial.print(SERVO_PIN[s]);
  Serial.print(F(")  dir "));   Serial.print(legDir(calLeg));
  Serial.print(F("  trim "));   Serial.println(trimDeg[s]);
}

void calSelect(int8_t step) { calLeg = (calLeg + 4 + step) & 3; calNod(); calReport(); }
void calTrim(int8_t fwdDeg) {
  uint8_t s = legServo(calLeg);
  int t = trimDeg[s] + legDir(calLeg) * fwdDeg;
  trimDeg[s] = (int8_t)constrain(t, -20, 20);
  writeLeg(calLeg, 0); calReport();
}
void calFlip() { uint8_t s = legServo(calLeg); servoDir[s] = -servoDir[s]; calShowForward(); calReport(); }

void enterCalib() {
  float z[4] = {0, 0, 0, 0};
  attachAll();
  startPairMove(false, 0, 600); while (!updateMove()) delay(5);   // rear pair, then front
  moveBlocking(z, 600);
  phase[0] = phase[1] = 0; segIdx = 0;
  state = ST_CALIB; calLeg = 0;
  Serial.println(F("CALIBRATION: 0-3 leg, +/- trim, f flip, t test, S save+exit, x cancel"));
  calNod();
}

void exitCalib(bool save, uint32_t now) {
  if (save) { saveConfig(); Serial.println(F("saved")); blink(4); }
  else      { loadConfig(); Serial.println(F("cancelled")); }
  float z[4] = {0, 0, 0, 0};
  moveBlocking(z, 300);
  waitRelease = true;
  standStill(now);
}

// ---- input -----------------------------------------------------------------------
uint8_t dirCommand(uint8_t b) {
  bool f = (b & B_FWD)  && !(b & B_BACK);
  bool k = (b & B_BACK) && !(b & B_FWD);
  bool l = (b & B_LEFT) && !(b & B_RIGHT);
  bool r = (b & B_RIGHT) && !(b & B_LEFT);
  if (f) return l ? CMD_ARC_FL : r ? CMD_ARC_FR : CMD_FWD;
  if (k) return l ? CMD_ARC_BL : r ? CMD_ARC_BR : CMD_BACK;
  return l ? CMD_SPIN_L : r ? CMD_SPIN_R : CMD_STOP;
}

void setSpeed(uint8_t s) {
  speedLevel = s;
  blink(s + 1);
  Serial.print(F("speed ")); Serial.println(s + 1);
}

// MODE (the stick's push button) while held, measured from the press
void modeHeld(uint32_t held, uint32_t now) {
  if (modeFired) return;
  if (state == ST_REST && held >= CAL_HOLD_MS)  { modeFired = true; enterCalib(); }
  else if (state == ST_CALIB && held >= HOLD_MS) { modeFired = true; exitCalib(true, now); }
  else if (state == ST_STAND && held >= HOLD_MS) { modeFired = true; beginSit(now); }
  else if (state == ST_WALK && held >= HOLD_MS)  { modeFired = true; serialCmd = CMD_STOP; sitPending = true; }
}

void modeTapped() {                   // released before anything fired
  if (state == ST_CALIB) calFlip();
  else if (state == ST_REST) wakeRequest = true;
  else if (inputSpeed() > 2) setSpeed((speedLevel + 1) % 3);   // inputs without a speed of their own
}

void onPress() {
  serialCmd = CMD_STOP;               // physical input takes over from serial
  if (state == ST_REST) { wakeRequest = true; waitRelease = true; }
}

void onRelease(uint8_t released) {    // calibration acts on release
  if (state != ST_CALIB || modeDown || waitRelease) return;
  if (released & B_LEFT)  calSelect(-1);
  if (released & B_RIGHT) calSelect(+1);
  if (released & B_FWD)   calTrim(+1);
  if (released & B_BACK)  calTrim(-1);
}

void scanInput(uint32_t now) {
  uint8_t r = inputMask(now);
  if (r != btnRaw) { btnRaw = r; btnRawSince = now; }
  if (r != btn && now - btnRawSince >= 25) {
    uint8_t pressed = r & ~btn, released = btn & ~r;
    btn = r; btnSince = now;
    if (pressed & 0x0F)  onPress();
    if (released & 0x0F) onRelease(released & 0x0F);
  }
  bool m = btn & B_MODE;
  if (m && !modeDown)  { modeDown = true; modeT0 = now; modeFired = false; }
  if (modeDown && m)   modeHeld(now - modeT0, now);
  if (modeDown && !m)  {
    modeDown = false; waitRelease = true;
    if (!modeFired && now - modeT0 >= 30) modeTapped();
  }
  if (waitRelease && btn == 0) waitRelease = false;

  if (m || waitRelease || state == ST_CALIB) btnCmd = CMD_STOP;
  else if (now - btnSince >= 70) btnCmd = dirCommand(btn);     // let diagonals settle
}

void printHelp() {
  Serial.println(F("w fwd  s back  a spin-left  d spin-right  q/e curve fwd  z/c curve back  x stop"));
  Serial.println(F("1 2 3 speed   [ ] stride   v save   r sit/wake   k calibrate   p status   j stick"));
}

void printStatus() {
  Serial.print(bodyReversed ? F("front = barrel-jack end  ") : F("front = USB end  "));
  Serial.print(F("state "));   Serial.print(state);
  Serial.print(F(" speed "));  Serial.print(speedLevel + 1);
  Serial.print(F(" stride ")); Serial.print(strideDeg, 0);
  Serial.print(F(" dir/trim"));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(' '); Serial.print(servoDir[i]); Serial.print('/'); Serial.print(trimDeg[i]);
  }
  Serial.println();
}

void setStride(float s) {
  strideDeg = constrain(s, STRIDE_MIN, STRIDE_MAX);
  Serial.print(F("stride ")); Serial.println(strideDeg, 0);
}

void scanSerial(uint32_t now) {
  while (Serial.available()) {
    char c = Serial.read();
    if (state == ST_CALIB) {
      switch (c) {
        case '0': case '1': case '2': case '3': calLeg = c - '0'; calNod(); calReport(); break;
        case '+': calTrim(+1); break;
        case '-': calTrim(-1); break;
        case 'f': calFlip(); break;
        case 't': calShowForward(); break;
        case 'S': exitCalib(true, now); break;
        case 'x': exitCalib(false, now); break;
      }
      continue;
    }
    uint8_t cmd = 255;
    switch (c) {
      case 'w': cmd = CMD_FWD; break;      case 's': cmd = CMD_BACK; break;
      case 'a': cmd = CMD_SPIN_L; break;   case 'd': cmd = CMD_SPIN_R; break;
      case 'q': cmd = CMD_ARC_FL; break;   case 'e': cmd = CMD_ARC_FR; break;
      case 'z': cmd = CMD_ARC_BL; break;   case 'c': cmd = CMD_ARC_BR; break;
      case 'x': case ' ': cmd = CMD_STOP; break;
      case '1': case '2': case '3': setSpeed(c - '1'); break;
      case '[': setStride(strideDeg - 1); break;
      case ']': setStride(strideDeg + 1); break;
      case 'r': if (state == ST_REST) wakeRequest = true;
                else if (state == ST_STAND) beginSit(now);
                else if (state == ST_WALK) { serialCmd = CMD_STOP; sitPending = true; }
                break;
      case 'k': if (state == ST_STAND || state == ST_REST) enterCalib(); break;
      case 'p': printStatus(); break;
      case 'v': saveConfig(); Serial.println(F("saved")); break;
      case '?': printHelp(); break;
      default:  inputSerial(c);
    }
    if (cmd != 255) {
      serialCmd = cmd;
      if (state == ST_REST && cmd != CMD_STOP) wakeRequest = true;
    }
  }
}

uint8_t wantedCommand() { return btnCmd != CMD_STOP ? btnCmd : serialCmd; }

// ---- setup / loop ------------------------------------------------------------------
void coreSetup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  loadConfig();
  loadOrientation();
  delay(300);                        // let the supply settle before the servos wake
  attachAll();                        // legs go to vertical, one at a time
  Serial.println(F("QuadBot-4 ready.  '?' for help."));
  uint32_t now = millis();
  uint8_t r = btn = btnRaw = inputMask(now);
  if (r & B_MODE) {                   // stick button held at power-up: calibration
    enterCalib();
    waitRelease = true; modeDown = true; modeFired = true; modeT0 = millis();
  } else {
    standStill(now);
    blink(speedLevel + 1);
  }
}

void coreLoop() {
  uint32_t now = millis();
  scanInput(now);
  scanSerial(now);
  now = millis();
  uint8_t want = wantedCommand();

  switch (state) {
    case ST_STAND:
      if (sitPending) { sitPending = false; beginSit(now); break; }
      if (want != CMD_STOP) {
        latchCommand(want);
        walkSpeed = currentSpeed();
        state = ST_WALK;
        startSegment();
      } else if (AUTO_REST_MS && now - idleSince > AUTO_REST_MS) {
        beginSit(now);
      }
      break;

    case ST_WALK:
      if (want != activeCmd) hurry = true;       // finish this half-cycle quickly
      if (updateMove()) {
        segIdx = (segIdx + 1) & 7;
        if ((segIdx & 3) == 0 && !latchCommand(want)) { standStill(now); break; }
        walkSpeed = currentSpeed();              // speed follows the stick, step by step
        startSegment();
      }
      break;

    case ST_SIT:  runSit(now);  break;
    case ST_WAKE: runWake(now); break;

    case ST_REST:
      if (wakeRequest || (want != CMD_STOP && !modeDown)) { wakeRequest = false; beginWake(now); }
      break;

    case ST_CALIB:
      break;                                     // handled in the input functions
  }
  updateLed(now);
  inputStatus(state, state == ST_WALK ? activeCmd : (uint8_t)CMD_STOP, walkSpeed);
}
