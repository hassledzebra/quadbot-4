/*  quadbot_matrix.h  --  status pictures for the Uno R4 WiFi's 12 x 8 LED matrix.
    Left 8 x 8: what the robot is doing.  Columns 9-11: speed bars.
    Top-right pixel: radio link (steady = good).                                    */
#pragma once
#include <Arduino.h>
#include "Arduino_LED_Matrix.h"

enum { PIC_STOP, PIC_F, PIC_FR, PIC_R, PIC_BR, PIC_B, PIC_BL, PIC_L, PIC_FL,
       PIC_REST, PIC_CAL, PIC_SEARCH, PIC_HEART, PIC_BARK, PIC_NOTE, PIC_COUNT };

// 8 rows x 8 columns each, '#' = lit
const char* const PICS[PIC_COUNT][8] = {
  {"........", "........", "..####..", "..#..#..", "..#..#..", "..####..", "........", "........"}, // stop
  {"...##...", "..####..", ".##..##.", "##.##.##", "...##...", "...##...", "...##...", "...##..."}, // forward
  {"..#####.", "....###.", "...##.#.", "..##..#.", ".##.....", "##......", "#.......", "........"}, // fwd-right curve
  {"..####..", ".#....#.", "#......#", "#....#.#", "#...###.", ".#..##..", "..#.#...", "........"}, // spin right
  {"#.......", "##......", ".##.....", "..##..#.", "...##.#.", "....###.", "..#####.", "........"}, // back-right curve
  {"...##...", "...##...", "...##...", "...##...", "##.##.##", ".##..##.", "..####..", "...##..."}, // back
  {".......#", "......##", ".....##.", ".#..##..", ".#.##...", ".###....", ".#####..", "........"}, // back-left curve
  {"..####..", ".#....#.", "#......#", "#.#....#", ".###...#", "..##..#.", "...#.#..", "........"}, // spin left
  {".#####..", ".###....", ".#.##...", ".#..##..", ".....##.", "......##", ".......#", "........"}, // fwd-left curve
  {"........", "####....", "...#....", "..#.....", ".#..###.", "####..#.", ".....#..", "....###."}, // resting (zz)
  {"........", "..####..", ".#....#.", ".#......", ".#......", ".#....#.", "..####..", "........"}, // calibration
  {"........", "........", "........", "...##...", "...##...", "........", "........", "........"}, // searching (blinks)
  {"........", ".##..##.", "########", "########", ".######.", "..####..", "...##...", "........"}, // heart (tricks)
  {"...##...", "...##...", "...##...", "...##...", "...##...", "........", "...##...", "...##..."}, // ! (bark)
  {"...##...", "...#.#..", "...#..#.", "...#....", "...#....", ".###....", "####....", ".##....."}, // note (wag, dance)
};

// cmd numbering matches CMD_* in quadbot_core.h
inline uint8_t picForCmd(uint8_t cmd) {
  static const uint8_t map_[9] = {PIC_STOP, PIC_F, PIC_B, PIC_L, PIC_R, PIC_FL, PIC_FR, PIC_BL, PIC_BR};
  return cmd < 9 ? map_[cmd] : (uint8_t)PIC_STOP;
}

ArduinoLEDMatrix ledMatrix;
uint8_t matrixFrame[8][12];   // board frame: row 0 along the digital header, column 0 at the USB end

// The pictures are drawn as seen by someone standing behind the robot, looking
// down: up = the way the robot walks.  matrixTurns says how that view sits on
// the board, in quarter turns:
//   0  view up = row 0 (digital-header edge)      landscape, 12 x 8
//   1  view up = column 11 (barrel-jack end)      portrait,   8 x 12
//   2  view up = row 7                            landscape
//   3  view up = column 0 (USB end)               portrait
// QuadBot-4 deck: USB end forward -> 3; body turned round (USB end at the
// rear) -> 1.  The robot sketch sets it; 0 keeps the original layout.
uint8_t matrixTurns = 0;

inline void matrixPut(uint8_t vr, uint8_t vc) {        // view row / column
  switch (matrixTurns & 3) {
    case 0: matrixFrame[vr][vc] = 1; break;
    case 1: matrixFrame[vc][11 - vr] = 1; break;
    case 2: matrixFrame[7 - vr][11 - vc] = 1; break;
    case 3: matrixFrame[7 - vc][vr] = 1; break;
  }
}

// speed: 0..2, or 255 for no bars.  link: draw the top-right link pixel.
inline void matrixShow(uint8_t pic, uint8_t speed, bool link) {
  for (uint8_t r = 0; r < 8; r++)
    for (uint8_t c = 0; c < 12; c++) matrixFrame[r][c] = 0;
  bool portrait = matrixTurns & 1;
  uint8_t top = portrait ? 4 : 0;                      // portrait: picture at the bottom
  for (uint8_t r = 0; r < 8; r++)
    for (uint8_t c = 0; c < 8; c++)
      if (PICS[pic][r][c] == '#') matrixPut(top + r, c);
  if (speed <= 2) {
    for (uint8_t k = 0; k <= speed; k++) {
      if (portrait) for (uint8_t r = 2 - k; r < 3; r++) matrixPut(r, 2 * k);       // 3 rows above the picture
      else          for (uint8_t r = 6 - 2 * k; r < 8; r++) matrixPut(r, 9 + k);   // 3 columns right of it
    }
  }
  if (link) matrixPut(0, portrait ? 7 : 11);
  ledMatrix.renderBitmap(matrixFrame, 8, 12);
}

// Battery gauge: an upright battery with 0-5 bars of fill (bottom up), in the
// same place as the pictures.  pct 0-100; blank = true draws nothing (blinking).
inline void matrixShowBattery(uint8_t pct, bool link, bool blank) {
  for (uint8_t r = 0; r < 8; r++)
    for (uint8_t c = 0; c < 12; c++) matrixFrame[r][c] = 0;
  if (!blank) {
    bool portrait = matrixTurns & 1;
    uint8_t top = portrait ? 4 : 0;
    static const char* const OUTLINE[8] =
      {"...##...", ".######.", ".#....#.", ".#....#.", ".#....#.", ".#....#.", ".#....#.", ".######."};
    for (uint8_t r = 0; r < 8; r++)
      for (uint8_t c = 0; c < 8; c++)
        if (OUTLINE[r][c] == '#') matrixPut(top + r, c);
    uint8_t bars = pct == 0 ? 0 : pct <= 20 ? 1 : pct <= 40 ? 2 : pct <= 60 ? 3 : pct <= 80 ? 4 : 5;
    for (uint8_t b = 0; b < bars; b++)
      for (uint8_t c = 2; c <= 5; c++) matrixPut(top + 6 - b, c);
    if (link) matrixPut(0, portrait ? 7 : 11);
  }
  ledMatrix.renderBitmap(matrixFrame, 8, 12);
}
