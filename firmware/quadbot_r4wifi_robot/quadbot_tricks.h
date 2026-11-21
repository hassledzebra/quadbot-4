/*  quadbot_tricks.h  --  QuadBot-4 tricks: wave, shake, bark, play bow, sit up,
    stretch, tail wag and a little dance.  For the four-servo robot (one hip
    servo per leg, leg v2), standing still, no walking.

    Each trick is a short list of key poses (FL, FR, RL, RR in degrees, + = foot
    forward, the same angles as the gait) played with the gait's own smooth
    move.  Nothing blocks: Wi-Fi, OTA and the battery check keep running, and
    pushing the stick (or any walk key, MODE, 'h', RESET) stops a trick and
    stands the robot up.  A trick asked for while resting or walking runs as
    soon as the robot is standing.

    Balance (body weight assumed at the middle of the body):
      * Four paws down (bark, bow, sit up, stretch, wag, dance): the body's
        centre stays 30 mm or more inside the paws.  Stable.  These tricks tip
        the body nose-up or nose-down, so some paws slide a little, like the
        existing sit.
      * One paw up (wave, shake): with every leg vertical the centre sits right
        on the line between the two diagonal paws (no margin).  So first all
        four legs swing LEAN degrees forward: the body slides about 25 mm toward
        the rear and the centre ends up 13 mm inside the three paws that stay
        down.  The raised paw then swings between 42 and 60 deg, 9 to 24 mm
        off the floor.  If the robot is nose-heavy (battery at the front) and
        tips toward the raised paw, raise LEAN to 30 (about 16 mm margin).

    Angles stay within the travel the sketch already uses (+-60; the sit uses
    55, the leg limit is 65).                                                   */
#pragma once
#include <Arduino.h>
#include "quadbot_core.h"
#include "quadbot_matrix.h"

const uint8_t ST_TRICK = 7;                 // after ST_MANUAL (6) in the sketch

constexpr int8_t LEAN = 25;                 // forward swing of all legs before a paw lifts
const uint16_t DEMO_GAP_MS = 1500;          // demo: standing pause between tricks

struct Key { int8_t a[4]; uint16_t ms; uint16_t hold; };   // FL FR RL RR, move time, then hold

enum { TR_NONE, TR_WAVE_R, TR_WAVE_L, TR_SHAKE, TR_BARK, TR_BOW, TR_SITUP, TR_STRETCH,
       TR_WAG, TR_DANCE, TR_COUNT };
const char* const TRICK_NAME[TR_COUNT] =
  {"", "wave R", "wave L", "shake", "bark", "bow", "sit up", "stretch", "wag", "dance"};

// ---- key poses ------------------------------------------------------------------
// Written for the RIGHT front paw; the left versions swap left and right legs.
const Key K_WAVE[] = {
  {{LEAN, LEAN, LEAN, LEAN}, 700, 200},     // lean back: weight off the front-right paw
  {{LEAN, 50, LEAN, LEAN},   300, 100},     // paw up
  {{LEAN, 60, LEAN, LEAN},   200,   0},     // wave: up ...
  {{LEAN, 42, LEAN, LEAN},   200,   0},     //       ... down (still 9 mm off the floor)
  {{LEAN, 60, LEAN, LEAN},   200,   0},
  {{LEAN, 42, LEAN, LEAN},   200,   0},
  {{LEAN, 60, LEAN, LEAN},   200,   0},
  {{LEAN, 42, LEAN, LEAN},   200,   0},
  {{LEAN, 60, LEAN, LEAN},   200, 300},
  {{LEAN, LEAN, LEAN, LEAN}, 350, 200},     // paw down
  {{0, 0, 0, 0},             700,   0},     // stand
};

const Key K_SHAKE[] = {                     // offer the paw, shake hands, hold, put it down
  {{LEAN, LEAN, LEAN, LEAN}, 700, 200},
  {{LEAN, 46, LEAN, LEAN},   350, 600},     // paw offered, 12 mm up
  {{LEAN, 51, LEAN, LEAN},   140,   0},
  {{LEAN, 42, LEAN, LEAN},   140,   0},
  {{LEAN, 51, LEAN, LEAN},   140,   0},
  {{LEAN, 42, LEAN, LEAN},   140,   0},
  {{LEAN, 51, LEAN, LEAN},   140,   0},
  {{LEAN, 46, LEAN, LEAN},   140, 700},
  {{LEAN, LEAN, LEAN, LEAN}, 350, 200},
  {{0, 0, 0, 0},             700,   0},
};

const Key K_BARK[] = {                      // "woof woof": a quick lunge, chest dipping
  {{-16, -16, -10, -10}, 110,  70},
  {{0, 0, 0, 0},         170, 160},
  {{-16, -16, -10, -10}, 110,  70},
  {{0, 0, 0, 0},         170, 600},
  {{-16, -16, -10, -10}, 110,  70},
  {{0, 0, 0, 0},         170, 160},
  {{-16, -16, -10, -10}, 110,  70},
  {{0, 0, 0, 0},         170,   0},
};

const Key K_BOW[] = {                       // play bow: chest down, rear up, tail end wiggles
  {{55, 55, 0, 0},     700, 500},
  {{55, 55, 12, -12},  180,   0},
  {{55, 55, -12, 12},  180,   0},
  {{55, 55, 12, -12},  180,   0},
  {{55, 55, -12, 12},  180,   0},
  {{55, 55, 12, -12},  180,   0},
  {{55, 55, 0, 0},     180, 500},
  {{0, 0, 0, 0},       700,   0},
};

const Key K_SITUP[] = {                     // dog sit: rear down (legs back), front up
  {{0, 0, -55, -55},  800, 900},
  {{8, -8, -55, -55}, 300, 300},            // looks left ...
  {{-8, 8, -55, -55}, 400, 300},            // ... and right
  {{0, 0, -55, -55},  300, 900},
  {{0, 0, 0, 0},      800,   0},
};

const Key K_STRETCH[] = {                   // front stretch, then back stretch
  {{60, 60, 0, 0},     900, 900},
  {{0, 0, 0, 0},       700, 200},
  {{0, 0, -60, -60},   900, 900},
  {{0, 0, 0, 0},       700,   0},
};

const Key K_WAG[] = {                       // tail wag: the rear end swings side to side
  {{0, 0, 15, -15},  180, 0},
  {{0, 0, -15, 15},  160, 0},
  {{0, 0, 15, -15},  160, 0},
  {{0, 0, -15, 15},  160, 0},
  {{0, 0, 15, -15},  160, 0},
  {{0, 0, -15, 15},  160, 0},
  {{0, 0, 15, -15},  160, 0},
  {{0, 0, -15, 15},  160, 0},
  {{0, 0, 0, 0},     180, 0},
};

const Key K_DANCE[] = {                     // twist, twist, bounce, bounce -- twice
  {{15, -15, 15, -15},  220, 0},
  {{-15, 15, -15, 15},  220, 0},
  {{15, -15, 15, -15},  220, 0},
  {{0, 0, 0, 0},        200, 0},
  {{20, 20, -20, -20},  200, 0},
  {{0, 0, 0, 0},        200, 0},
  {{20, 20, -20, -20},  200, 0},
  {{0, 0, 0, 0},        200, 150},
  {{-15, 15, -15, 15},  220, 0},
  {{15, -15, 15, -15},  220, 0},
  {{-15, 15, -15, 15},  220, 0},
  {{0, 0, 0, 0},        200, 0},
  {{20, 20, -20, -20},  200, 0},
  {{0, 0, 0, 0},        200, 0},
  {{20, 20, -20, -20},  200, 0},
  {{0, 0, 0, 0},        250, 0},
};

#define N_KEYS(k) ((uint8_t)(sizeof(k) / sizeof(k[0])))

// ---- player -------------------------------------------------------------------------
uint8_t   trick = TR_NONE, trickPending = TR_NONE, trickPic = PIC_HEART;
const Key* trKeys = nullptr;
uint8_t   trCount = 0, trIdx = 0;
bool      trMirror = false, trHolding = false;
uint32_t  trHoldT0 = 0;
bool      demoOn = false;
uint8_t   demoNext = TR_WAVE_R;

void trickKey() {
  const Key& k = trKeys[trIdx];
  float t[4];
  for (uint8_t i = 0; i < 4; i++) t[i] = k.a[trMirror ? (i ^ 1) : i];   // FL<->FR, RL<->RR
  startMove(t, k.ms);
  trHolding = false;
}

void playTrick(uint8_t id, const Key* keys, uint8_t n, bool mirror, uint8_t pic) {
  trick = id; trKeys = keys; trCount = n; trIdx = 0; trMirror = mirror; trickPic = pic;
  hurry = false; sitPending = false;
  state = ST_TRICK;
  Serial.print(F("trick: ")); Serial.println(TRICK_NAME[id]);
  trickKey();
}

// one function per trick
void trickWave(bool left) { playTrick(left ? TR_WAVE_L : TR_WAVE_R, K_WAVE, N_KEYS(K_WAVE), left, PIC_HEART); }
void trickShake()         { playTrick(TR_SHAKE,   K_SHAKE,   N_KEYS(K_SHAKE),   false, PIC_HEART); }
void trickBark()          { playTrick(TR_BARK,    K_BARK,    N_KEYS(K_BARK),    false, PIC_BARK); }
void trickBow()           { playTrick(TR_BOW,     K_BOW,     N_KEYS(K_BOW),     false, PIC_HEART); }
void trickSitUp()         { playTrick(TR_SITUP,   K_SITUP,   N_KEYS(K_SITUP),   false, PIC_HEART); }
void trickStretch()       { playTrick(TR_STRETCH, K_STRETCH, N_KEYS(K_STRETCH), false, PIC_HEART); }
void trickWag()           { playTrick(TR_WAG,     K_WAG,     N_KEYS(K_WAG),     false, PIC_NOTE); }
void trickDance()         { playTrick(TR_DANCE,   K_DANCE,   N_KEYS(K_DANCE),   false, PIC_NOTE); }

void startTrick(uint8_t id) {
  switch (id) {
    case TR_WAVE_R:  trickWave(false); break;
    case TR_WAVE_L:  trickWave(true);  break;
    case TR_SHAKE:   trickShake();     break;
    case TR_BARK:    trickBark();      break;
    case TR_BOW:     trickBow();       break;
    case TR_SITUP:   trickSitUp();     break;
    case TR_STRETCH: trickStretch();   break;
    case TR_WAG:     trickWag();       break;
    case TR_DANCE:   trickDance();     break;
  }
}

// Ask for a trick from any state; it starts once the robot stands still.
void requestTrick(uint8_t id) {
  if (id == TR_NONE || id >= TR_COUNT || state == ST_CALIB) return;
  trickPending = id;
  if (state == ST_REST) wakeRequest = true;
  else if (state == ST_WALK) serialCmd = CMD_STOP;      // stops at the next all-legs-vertical point
}

// Stop at once: legs back to vertical, stand.  Also ends the demo.
void trickStop(uint32_t now) {
  demoOn = false; trickPending = TR_NONE;
  if (state != ST_TRICK) return;
  trick = TR_NONE;
  float z[4] = {0, 0, 0, 0};
  moveBlocking(z, 500);
  standStill(now);
  Serial.println(F("trick stopped"));
}

void setDemo(bool on) {
  demoOn = on;
  if (on) { demoNext = TR_WAVE_R; if (state == ST_REST) wakeRequest = true; }
  else trickPending = TR_NONE;
  Serial.println(on ? F("demo on: every trick in turn ('T' again to stop)") : F("demo off"));
}

void printTricks() {
  Serial.println(F("tricks: t1 wave right  t2 wave left  t3 shake  t4 bark  t5 play bow"));
  Serial.println(F("        t6 sit up  t7 stretch  t8 tail wag  t9 dance   T demo on/off"));
}

// Call every loop, before coreLoop().
void trickLoop(uint32_t now) {
  if (trickPending && state == ST_REST) wakeRequest = true;
  if (demoOn && !trickPending && state == ST_STAND && now - idleSince > DEMO_GAP_MS) {
    trickPending = demoNext;
    demoNext = demoNext + 1 < TR_COUNT ? demoNext + 1 : TR_WAVE_R;
  }
  if (trickPending && state == ST_STAND && !moving) {
    uint8_t id = trickPending; trickPending = TR_NONE;
    startTrick(id);
    return;
  }
  if (state != ST_TRICK) return;
  if (wantedCommand() != CMD_STOP || (btn & B_MODE)) { trickStop(now); return; }   // stick takes over
  if (!updateMove()) return;
  if (!trHolding) { trHolding = true; trHoldT0 = now; }
  if (now - trHoldT0 < trKeys[trIdx].hold) return;
  if (++trIdx < trCount) trickKey();
  else { trick = TR_NONE; standStill(now); }
}
