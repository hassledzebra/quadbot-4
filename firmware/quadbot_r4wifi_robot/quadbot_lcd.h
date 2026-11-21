/*  quadbot_lcd.h  --  optional 16 x 2 character LCD (HD44780).  No library needed.
    Works with either kind:

    A) LCD with an I2C backpack (4-pin board soldered on the back, PCF8574):
         GND -> GND   VCC -> 5V   SDA -> SDA (A4)   SCL -> SCL (A5)
       Found by itself at 0x27 or 0x3F.  It shares the bus with the PCA9685
       servo board (0x40).  Blank screen: turn the contrast screw on the back.

    B) Plain 16-pin LCD, as in the Arduino starter kits (4-bit mode):
         1 VSS -> GND          2 VDD -> 5V          3 V0  -> 10 k pot middle
                                                       (pot ends to 5V and GND)
         4 RS  -> D2           5 RW  -> GND         6 E   -> D3
         11 D4 -> D4           12 D5 -> D5          13 D6 -> D6     14 D7 -> D7
         15 A  -> 220 ohm -> 5V                     16 K  -> GND
       Pins 7-10 stay empty.  A plain LCD cannot answer back, so the sketch
       always drives D2-D7 for it when no I2C LCD is found; set LCD_PARALLEL to
       false if those pins are needed for something else.
       Blank screen or a row of solid blocks: turn the pot.

    Writes only the characters that changed, a few at a time, so the gait never
    waits more than a few milliseconds for the display.                          */
#pragma once
#include <Arduino.h>
#include <Wire.h>

const bool    LCD_PARALLEL = true;
const uint8_t LCD_P_RS = 2, LCD_P_E = 3;
const uint8_t LCD_P_D[4] = {4, 5, 6, 7};             // D4..D7 of the LCD

const uint8_t LCD_ADDRS[] = {0x27, 0x3F, 0x26, 0x20, 0x38};
const uint8_t LCD_RS = 0x01, LCD_EN = 0x04, LCD_BL = 0x08;   // PCF8574 backpack bits

enum { LCD_NONE, LCD_I2C, LCD_PAR };
uint8_t lcdKind = LCD_NONE;
uint8_t lcdAddr = 0;
char    lcdShown[2][17], lcdWant[2][17];

// I2C needs pull-up resistors, which the LCD backpack and the PCA9685 board
// carry.  With nothing plugged in the pins float, and a scan can stall, so check
// for pull-ups first: pull the line low, let go, and see whether it comes back up.
inline bool lcdPullupsPresent() {
  const uint8_t pins[2] = {A4, A5};                  // the Uno R4's Wire pins (SDA, SCL)
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(pins[i], OUTPUT); digitalWrite(pins[i], LOW); delayMicroseconds(50);
    pinMode(pins[i], INPUT);  delayMicroseconds(30);
    bool up = digitalRead(pins[i]);
    if (!up) return false;
  }
  return true;
}

// ---- the two ways of sending 4 bits ------------------------------------------------
inline void lcdParNibble(uint8_t n, bool rs) {
  digitalWrite(LCD_P_RS, rs);
  for (uint8_t b = 0; b < 4; b++) digitalWrite(LCD_P_D[b], (n >> b) & 1);
  digitalWrite(LCD_P_E, HIGH); delayMicroseconds(1);
  digitalWrite(LCD_P_E, LOW);  delayMicroseconds(1);
}

inline void lcdSend(uint8_t v, bool rs) {            // one byte as two 4-bit transfers
  if (lcdKind == LCD_PAR) {
    lcdParNibble(v >> 4, rs);
    lcdParNibble(v & 0x0F, rs);
    delayMicroseconds(45);                           // the LCD needs 37 us per byte
  } else if (lcdKind == LCD_I2C) {
    uint8_t mode = rs ? LCD_RS : 0;
    uint8_t hi = (v & 0xF0) | mode | LCD_BL, lo = ((v << 4) & 0xF0) | mode | LCD_BL;
    Wire.beginTransmission(lcdAddr);                 // each I2C byte takes longer than 37 us
    Wire.write(hi | LCD_EN); Wire.write(hi);
    Wire.write(lo | LCD_EN); Wire.write(lo);
    Wire.endTransmission();
  }
}
inline void lcdCmd(uint8_t c) { lcdSend(c, false); }
inline void lcdChar(char ch)  { lcdSend((uint8_t)ch, true); }

inline void lcdInitNibble(uint8_t n) {               // start-up: 4 bits on their own
  if (lcdKind == LCD_PAR) { lcdParNibble(n, false); return; }
  Wire.beginTransmission(lcdAddr);
  Wire.write((n << 4) | LCD_BL | LCD_EN); Wire.write((n << 4) | LCD_BL);
  Wire.endTransmission();
}

inline bool lcdFindI2c() {
  if (!lcdPullupsPresent()) return false;
  Wire.begin();
  Wire.setClock(100000);
  for (uint8_t a : LCD_ADDRS) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { lcdAddr = a; return true; }
  }
  return false;
}

inline bool lcdBegin() {
  if (lcdFindI2c()) lcdKind = LCD_I2C;
  else if (LCD_PARALLEL) {
    lcdKind = LCD_PAR;
    pinMode(LCD_P_RS, OUTPUT); pinMode(LCD_P_E, OUTPUT);
    digitalWrite(LCD_P_E, LOW);
    for (uint8_t b = 0; b < 4; b++) pinMode(LCD_P_D[b], OUTPUT);
  } else return false;
  delay(50);
  lcdInitNibble(0x03); delay(5);                     // HD44780 4-bit start-up
  lcdInitNibble(0x03); delay(1);
  lcdInitNibble(0x03); delay(1);
  lcdInitNibble(0x02); delay(1);
  lcdCmd(0x28);                                      // 2 lines, 5x8
  lcdCmd(0x0C);                                      // display on, no cursor
  lcdCmd(0x01); delay(2);                            // clear
  lcdCmd(0x06);                                      // left to right
  for (uint8_t r = 0; r < 2; r++) {
    memset(lcdShown[r], ' ', 16); lcdShown[r][16] = 0;
    memset(lcdWant[r], ' ', 16);  lcdWant[r][16] = 0;
  }
  return true;
}

// Set what a line should say (padded / cut to 16 characters).
inline void lcdSet(uint8_t row, const char* s) {
  uint8_t i = 0;
  for (; i < 16 && s[i]; i++) lcdWant[row][i] = s[i];
  for (; i < 16; i++) lcdWant[row][i] = ' ';
}

// Send up to 'budget' changed characters.  Call every loop or on a timer.
inline void lcdFlush(uint8_t budget = 8) {
  if (lcdKind == LCD_NONE) return;
  for (uint8_t r = 0; r < 2 && budget; r++) {
    int8_t cursor = -1;
    for (uint8_t c = 0; c < 16 && budget; c++) {
      if (lcdWant[r][c] == lcdShown[r][c]) { cursor = -1; continue; }
      if (cursor != c) lcdCmd(0x80 | (r ? 0x40 : 0) | c);
      lcdChar(lcdWant[r][c]);
      lcdShown[r][c] = lcdWant[r][c];
      cursor = c + 1; budget--;
    }
  }
}

inline void lcdPrintKind() {
  if (lcdKind == LCD_I2C) { Serial.print(F("LCD: I2C at 0x")); Serial.println(lcdAddr, HEX); }
  else if (lcdKind == LCD_PAR) Serial.println(F("LCD: plain 16-pin on D2-D7 (no I2C LCD found)"));
}
