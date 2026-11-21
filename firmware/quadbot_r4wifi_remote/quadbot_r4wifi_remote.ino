/*
  QuadBot-4  --  joystick remote for an Arduino Uno R4 WiFi
  =========================================================
  Joystick module (KY-023 type) on this board:
      GND -> GND     +5V -> 5V     VRx -> A0     VRy -> A1     SW -> D2
  Power it from a USB power bank (USB-C) or a 9 V battery on the barrel jack.

  It joins the robot's Wi-Fi network and sends the stick 25 times a second.
  LED matrix:  moving dot  = looking for the robot
               arrow       = what the stick is asking for, bars = speed
               zz / C      = robot is resting / in calibration
               top-right dot lit = robot is answering

  Stick set-up (once, saved on this board): hold the stick pressed while
  switching on, let go, then push FORWARD (forward arrow blinks) and let go,
  then push RIGHT (right arrow blinks) and let go.
*/
#include <Arduino.h>
#include <WiFiS3.h>
#include <EEPROM.h>
#include "quadbot_stick.h"
#include "quadbot_matrix.h"

const char      ROBOT_SSID[] = "QuadBot-4";
const char      ROBOT_PASS[] = "walk4legs";       // must match the robot sketch
const IPAddress ROBOT_IP(192, 168, 4, 1);
const uint16_t  UDP_PORT     = 4210;
const uint8_t   JOY_X_PIN = A0, JOY_Y_PIN = A1, JOY_SW_PIN = 2;
const uint8_t   ROBOT_REST = 3, ROBOT_CALIB = 5;  // state numbers sent by the robot

WiFiUDP  udp;
StickMap joyMap;
int      joyCx = 512, joyCy = 512;
uint8_t  seq = 0, robotState = 0;
int8_t   sentX = 0, sentY = 0;
bool     sentSw = false;
uint32_t lastSend = 0, lastReply = 0, lastShow = 0;

void readStick(int8_t &x, int8_t &y, bool &sw) {
  int8_t a = stickScale(analogRead(JOY_X_PIN), joyCx);
  int8_t b = stickScale(analogRead(JOY_Y_PIN), joyCy);
  stickApply(joyMap, a, b, x, y);
  sw = digitalRead(JOY_SW_PIN) == LOW;
}

uint8_t picForMask(uint8_t m) {
  bool f = m & B_FWD, b = m & B_BACK, l = m & B_LEFT, r = m & B_RIGHT;
  if (f) return l ? PIC_FL : r ? PIC_FR : PIC_F;
  if (b) return l ? PIC_BL : r ? PIC_BR : PIC_B;
  return l ? PIC_L : r ? PIC_R : PIC_STOP;
}

void connectWifi() {
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ROBOT_SSID, ROBOT_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 6000) {
      uint8_t f = (millis() / 150) % 4;            // a dot walking round the centre
      matrixShow(PIC_SEARCH, 255, false);
      matrixFrame[f < 2 ? 2 : 5][f == 0 || f == 3 ? 2 : 5] = 1;
      ledMatrix.renderBitmap(matrixFrame, 8, 12);
      delay(150);
    }
  }
  udp.begin(UDP_PORT + 1);
  Serial.print(F("connected to ")); Serial.println(ROBOT_SSID);
}

void sendStick(int8_t x, int8_t y, bool sw) {
  uint8_t p[7] = {'Q', '4', ++seq, (uint8_t)x, (uint8_t)y, (uint8_t)(sw ? 1 : 0), 0};
  p[6] = p[0] ^ p[1] ^ p[2] ^ p[3] ^ p[4] ^ p[5];
  udp.beginPacket(ROBOT_IP, UDP_PORT);
  udp.write(p, sizeof p);
  udp.endPacket();
  sentX = x; sentY = y; sentSw = sw;
}

void readReplies(uint32_t now) {
  int n = udp.parsePacket();
  if (n <= 0) return;
  uint8_t r[8];
  int len = udp.read(r, sizeof r);
  if (len == 5 && r[0] == 'Q' && r[1] == 'R' && (uint8_t)(r[0] ^ r[1] ^ r[2] ^ r[3]) == r[4]) {
    robotState = r[2]; lastReply = now;
  }
}

void setup() {
  Serial.begin(115200);
  ledMatrix.begin();
  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  long sx = 0, sy = 0;
  for (uint8_t i = 0; i < 16; i++) { sx += analogRead(JOY_X_PIN); sy += analogRead(JOY_Y_PIN); delay(2); }
  sx /= 16; sy /= 16;
  if (sx > 380 && sx < 644) joyCx = sx;
  if (sy > 380 && sy < 644) joyCy = sy;
  joyMap = stickMapLoad();
  if (digitalRead(JOY_SW_PIN) == LOW) {                    // stick set-up
    while (digitalRead(JOY_SW_PIN) == LOW) { matrixShow(PIC_STOP, 255, false); delay(20); }
    delay(50);
    joyMap = stickLearn(JOY_X_PIN, JOY_Y_PIN, joyCx, joyCy, [](uint8_t step) {
      bool on = (millis() % 400) < 250;
      matrixShow(step == 0 ? PIC_F : step == 1 ? PIC_R : PIC_STOP, 255, false);
      if (!on && step < 2) { for (auto &row : matrixFrame) for (auto &px : row) px = 0; ledMatrix.renderBitmap(matrixFrame, 8, 12); }
    });
    Serial.println(F("stick set-up saved"));
  }
  connectWifi();
}

void loop() {
  uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED) { connectWifi(); now = millis(); }
  int8_t x, y; bool sw;
  readStick(x, y, sw);
  bool changed = sw != sentSw || abs(x - sentX) > 4 || abs(y - sentY) > 4;
  if (now - lastSend >= 40 || (changed && now - lastSend >= 15)) { lastSend = now; sendStick(x, y, sw); }
  readReplies(now);
  if (now - lastShow >= 80) {
    lastShow = now;
    bool link = now - lastReply < 600;
    uint8_t m = stickMask(x, y, sw);
    uint8_t pic = picForMask(m);
    if (link && robotState == ROBOT_REST)  pic = PIC_REST;
    if (link && robotState == ROBOT_CALIB) pic = PIC_CAL;
    matrixShow(pic, (m & 0x0F) ? stickSpeed(x, y) : 255, link);
  }
}
