/*
  QuadBot-4  --  robot sketch for the Arduino Uno R4 WiFi (wireless control)
  =========================================================================
  The robot's Uno R4 WiFi makes its own Wi-Fi network, "QuadBot-4".  Two ways
  to drive it:
    1. The remote: a second Uno R4 WiFi with the joystick, running
       quadbot_r4wifi_remote.ino.  It joins the network and sends the stick
       position 25 times a second.
    2. A phone: join "QuadBot-4", open http://192.168.4.1.  v3c: the page has
       tricks, stand / lie down and leg by leg; no walking pad for now.
  If the link drops for 0.3 s (0.7 s from a phone) the robot stops at its
  next all-legs-vertical point.

  Wiring is the same as the wired version, minus the joystick tether:
    servos D8, D9, D10, D11 on the 5 V buck converter rail;
    battery to the barrel jack; VIN -> buck IN+, GND -> buck IN-.
  Front and back: this robot is built with the body turned round, so the
  barrel-jack end of the Uno is the FRONT (BODY_REVERSED_DEFAULT in
  quadbot_core.h).  Legs: FL = D11, FR = D10, RL = D9, RR = D8.  Serial 'F',
  or "swap front / back" on the phone page, switches it (saved on the board).

  The LED matrix shows what the robot is doing (arrow, square = standing,
  zz = resting, C = calibration), drawn as you see it standing behind the
  robot: the forward arrow points the way the robot walks.  Speed bars sit
  above the picture and the top-right dot is lit while a controller is
  connected.  If the pictures are turned the wrong way on your board, serial
  'm' or "turn display" on the phone page turns them 90 deg (saved).

  Wireless updates (needs the ArduinoOTA library by Juraj Andrassy; set QB_OTA
  to 0 below to build without it): the robot accepts a new sketch over Wi-Fi.
  Join "QuadBot-4", then
    curl --data-binary @quadbot_r4wifi_robot.ino.bin \
         http://arduino:walk4legs@192.168.4.1:65280/sketch
  The first upload of this version must still go over USB.

  Battery (quadbot_batt.h): a 10 k / 4.7 k divider from the 2S LiPo (+) to A0.
  The phone page shows the voltage and charge; the LED matrix shows a battery
  gauge for 2 s of every 8 while standing, flashes it when LOW (6.9 V), and
  when EMPTY (6.6 V) the robot sits down and will not walk until a charged
  battery is fitted.  Serial 'b' prints it; "B 7.95" calibrates it to a
  multimeter reading.  Nothing wired to A0: the sketch behaves as before.

  Optional 16 x 2 LCD (quadbot_lcd.h, wiring there): either one with an I2C
  backpack (VCC 5V, GND, SDA, SCL; found by itself) or a plain 16-pin starter-
  kit LCD on D2-D7 with a 10 k contrast pot.  Line 1 is the battery, line 2
  what the robot is doing.

  Same stick actions as the wired version: push to walk (further = faster),
  press and hold 1 s to sit / press to stand, and while sitting hold the
  press for 3 s for leg calibration.

  Tricks (quadbot_tricks.h, v3c): wave right / left, shake, bark, play bow,
  sit up, stretch, tail wag, dance.  Phone page: "tricks".  Serial: t1 .. t9,
  t0 lists them, T runs them all in turn (demo) until T again.  Pushing the
  stick, MODE, 'h' or RESET stops a trick and the robot stands.
  The phone page has no walking pad in v3c (the gait is not working yet);
  walking is still there in the code, for the joystick remote and serial w/s/a/d.
*/
#include <Arduino.h>
#include <WiFiS3.h>
// Wi-Fi updates need the ArduinoOTA library (Juraj Andrassy, Library Manager).
// Without it, set QB_OTA to 0.  (A __has_include test does not work here:
// arduino-cli evaluates it before it adds the library, so OTA silently drops out.)
#define QB_OTA 1
#if QB_OTA
#include <ArduinoOTA.h>
#endif
#include "quadbot_core.h"
#include "quadbot_matrix.h"
#include "quadbot_batt.h"
#include "quadbot_lcd.h"
#include "quadbot_tricks.h"

const char     AP_SSID[]   = "QuadBot-4";
const char     AP_PASS[]   = "walk4legs";   // 8 characters or more; same in the remote
const char     OTA_PASS[]  = "walk4legs";   // user name "arduino"
const uint16_t UDP_PORT    = 4210;
const uint16_t LINK_MS     = 300;           // remote silent this long -> stop
const uint16_t PHONE_MS    = 700;           // phone silent this long  -> stop
const uint8_t  ST_MANUAL   = 6;             // legs driven one by one from the page; gait paused

WiFiUDP    udp;
WiFiServer web(80);
bool     wifiOk = false, fromPhone = false;
int8_t   linkX = 0, linkY = 0;
bool     linkSw = false;
uint32_t lastPacket = 0, lastUdpPoll = 0, lastWebPoll = 0, lastMatrix = 0;
uint8_t  replyCount = 0;
uint8_t  shownState = ST_STAND, shownCmd = CMD_STOP, shownSpeed = 0;

// ---- matrix orientation ----------------------------------------------------------
// The deck holds the Uno with its USB end at one end of the body.  With the USB
// end at the front the view's "up" is column 0 (3 quarter turns); with the body
// turned round it is column 11 (1).  matrixFix is a user correction on top,
// in quarter turns, saved in EEPROM bytes 14-15.
const uint8_t EE_MATRIX_MAGIC = 0x6D;
uint8_t matrixFix = 0;

void applyMatrixTurns() { matrixTurns = ((bodyReversed ? 1 : 3) + matrixFix) & 3; }
void loadMatrixFix() {
  matrixFix = EEPROM.read(14) == EE_MATRIX_MAGIC ? (EEPROM.read(15) & 3) : 0;
  applyMatrixTurns();
}
void turnMatrix() {
  matrixFix = (matrixFix + 1) & 3;
  EEPROM.update(14, EE_MATRIX_MAGIC);
  EEPROM.update(15, matrixFix);
  applyMatrixTurns();
  lastMatrix = 0;                                // redraw now
  Serial.print(F("display turned, correction ")); Serial.print(matrixFix * 90); Serial.println(F(" deg"));
}
void swapFrontBack() {
  if (setBodyReversed(!bodyReversed)) {
    applyMatrixTurns(); lastMatrix = 0;
    Serial.println(bodyReversed ? F("front = barrel-jack end") : F("front = USB end"));
  } else Serial.println(F("stand still first"));
}

// Phone page.  v3c: no walking pad (the gait is not working yet).  Tricks first,
// then sit / stand, RESET, leg by leg and setup.  The page still uses /j for the
// sit / stand button (x = y = 0), so the robot never walks from the phone.
const char PAGE[] =
  "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
  "<title>QuadBot-4</title><style>body{font-family:sans-serif;background:#12161b;color:#e8ecf0;"
  "display:flex;flex-direction:column;align-items:center;gap:16px;margin:0;padding:24px;"
  "user-select:none;-webkit-user-select:none}"
  "button{border-radius:16px;border:1px solid #39424d;background:#232b34;"
  "color:#e8ecf0;touch-action:none}button.on{background:#f07f1e;color:#1b1206}"
  "details{width:292px}summary{font-size:16px;cursor:pointer}.n{font-size:13px;color:#aab3bd;margin:8px 0 12px}"
  "#l{display:flex;flex-direction:column;gap:14px}.r{display:flex;align-items:center;gap:8px;font-size:13px}"
  ".r span{width:70px}.r input{flex:1}.r b{width:36px;text-align:right}"
  ".w{display:flex;gap:8px;margin-top:14px}.w button{flex:1;height:44px;font-size:15px}"
  "#p{width:292px;height:60px;font-size:17px}"
  "#rs{width:292px;height:52px;font-size:17px;font-weight:bold;background:#1f4e7a}"
  "#bt{font-size:15px;color:#aab3bd;text-align:center;max-width:292px}#bt.w{color:#f0b429}"
  "#bt.e{color:#ff5a4f;font-weight:bold}.tk{display:grid;grid-template-columns:repeat(3,92px);gap:8px}"
  ".tk button{height:64px;font-size:16px}.tk .x{background:#3a2326}"
  "</style></head><body><h2 style=margin-bottom:0>QuadBot-4</h2><div id=bt>battery ...</div>"
  "<div class=tk>"
  "<button data-t=1>wave R</button><button data-t=2>wave L</button><button data-t=3>shake</button>"
  "<button data-t=4>bark</button><button data-t=5>play bow</button><button data-t=6>sit up</button>"
  "<button data-t=7>stretch</button><button data-t=8>tail wag</button><button data-t=9>dance</button>"
  "<button data-t=10>demo</button><button class=x data-t=0>stop</button></div>"
  "<button id=p>tap = stand up &middot; hold 1 s = lie down</button>"
  "<button id=rs>RESET &middot; all legs vertical</button>"
  "<details><summary>leg by leg</summary><p class=n>Each slider moves one leg: 0 = vertical, + = foot forward. "
  "Tricks are paused while you use them; tap <i>done</i> when finished.</p><div id=l></div>"
  "<div class=w><button id=lc>all vertical</button><button id=lx>done</button></div></details>"
  "<details><summary>setup</summary><p class=n>Swap front / back if the tricks use the wrong end: play bow "
  "should lower the barrel-jack end. Turn display until the heart on the board is upright. Both are saved on the robot.</p>"
  "<div class=w><button id=of>swap front / back</button><button id=om>turn display</button></div></details><script>"
  "let s=0,busy=0,last=0;"
  "function send(){if(busy)return;busy=1;fetch('/j?x=0&y=0&s='+s).catch(()=>{}).finally(()=>busy=0)}"
  "setInterval(()=>{if(s||Date.now()-last<600)send()},150);"
  "const p=document.getElementById('p');"
  "p.onpointerdown=e=>{p.setPointerCapture(e.pointerId);p.classList.add('on');s=1;send()};"
  "p.onpointerup=p.onpointercancel=()=>{p.classList.remove('on');s=0;last=Date.now()};"
  "const L=['front-left','front-right','rear-left','rear-right'],$=i=>document.getElementById(i);"
  "L.forEach((n,i)=>$('l').insertAdjacentHTML('beforeend','<label class=r><span>'+n+'</span>"
  "<input type=range min=-60 max=60 step=5 value=0 data-l='+i+'><b id=g'+i+'>0</b></label>'));"
  "let lb=0,lq=null;function lpush(q){lq=q;lflush()}"
  "function lflush(){if(lb||!lq)return;const q=lq;lq=null;lb=1;fetch('/l?'+q).catch(()=>{}).finally(()=>{lb=0;lflush()})}"
  "document.querySelectorAll('[data-l]').forEach(r=>r.oninput=()=>{$('g'+r.dataset.l).textContent=r.value;"
  "lpush('i='+r.dataset.l+'&a='+r.value)});"
  "function lzero(){document.querySelectorAll('[data-l]').forEach(r=>{r.value=0;$('g'+r.dataset.l).textContent=0})}"
  "$('lc').onclick=()=>{lzero();lpush('i=4&a=0')};"
  "$('lx').onclick=()=>{lzero();lpush('x=1')};"
  "$('rs').onclick=()=>{s=0;last=Date.now();document.querySelectorAll('.on').forEach(b=>b.classList.remove('on'));"
  "lzero();lpush('r=1')};"
  "$('of').onclick=()=>fetch('/o?f=1').catch(()=>{});$('om').onclick=()=>fetch('/o?m=1').catch(()=>{});"
  "document.querySelectorAll('[data-t]').forEach(b=>b.onclick=()=>fetch('/t?n='+b.dataset.t).catch(()=>{}));"
  "function bat(){fetch('/b').then(r=>r.text()).then(t=>{const i=t.indexOf('|');$('bt').className=t.slice(0,i);"
  "$('bt').textContent=t.slice(i+1)}).catch(()=>{})}bat();setInterval(bat,3000);"
  "</script></body></html>";

// ---- battery: safety, LCD and status text ------------------------------------------
// EMPTY: stop walking, sit down and stay down (the servos are off while resting).
void battGuard(uint32_t now) {
  if (battUpdate(now)) { battPrint(); lastMatrix = 0; }
  if (battState != BATT_EMPTY) return;
  if (demoOn || trickPending) { demoOn = false; trickPending = TR_NONE; }
  if (state == ST_TRICK)      trickStop(now);              // stands; the next pass sits it down
  else if (state == ST_MANUAL) legReset(now);
  else if (state == ST_CALIB) exitCalib(false, now);
  else if (state == ST_WALK)  { serialCmd = CMD_STOP; sitPending = true; }
  else if (state == ST_STAND) beginSit(now);
}

// "7.62" into buf
void fmtVolts(char* buf, size_t n) {
  int cv = (int)(battV * 100 + 0.5f);
  snprintf(buf, n, "%d.%02d", cv / 100, cv % 100);
}

// For the phone page: "class|text"
void battMessage(char* out, size_t n) {
  char v[8]; fmtVolts(v, sizeof v);
  switch (battState) {
    case BATT_OK:    snprintf(out, n, "|battery %s V - %u %%", v, battPercent()); break;
    case BATT_LOW:   snprintf(out, n, "w|battery LOW %s V - %u %% - charge soon", v, battPercent()); break;
    case BATT_EMPTY: snprintf(out, n, "e|battery EMPTY %s V - robot sat down. Switch off and charge.", v); break;
    default:         snprintf(out, n, "|battery not measured"); break;
  }
}

const char* const STATE_NAME[8] = {"STAND", "WALK", "SIT", "REST", "WAKE", "CALIB", "LEGS", "TRICK"};
const char* const CMD_NAME[9]   = {"stop", "fwd", "back", "spin L", "spin R", "fwd L", "fwd R", "back L", "back R"};
const char* const LEG_NAME[4]   = {"FL", "FR", "RL", "RR"};

void updateLcd(uint8_t st, uint8_t cmd, uint8_t speed, bool link) {
  if (lcdKind == LCD_NONE) return;
  char line[24], v[8];
  fmtVolts(v, sizeof v);
  switch (battState) {
    case BATT_OK:    snprintf(line, sizeof line, "Batt %sV %4u%%", v, battPercent()); break;
    case BATT_LOW:   snprintf(line, sizeof line, "LOW  %sV %4u%%", v, battPercent()); break;
    case BATT_EMPTY: snprintf(line, sizeof line, "BATT EMPTY %sV", v); break;
    default:         snprintf(line, sizeof line, "Batt: no sensor"); break;
  }
  lcdSet(0, line);
  char extra[8] = "";
  if (st == ST_WALK && cmd < 9)  snprintf(extra, sizeof extra, "%s", CMD_NAME[cmd]);
  else if (st == ST_REST)        snprintf(extra, sizeof extra, "zz");
  else if (st == ST_CALIB)       snprintf(extra, sizeof extra, "leg %s", LEG_NAME[calLeg & 3]);
  else if (st == ST_TRICK && trick < TR_COUNT) snprintf(extra, sizeof extra, "%s", TRICK_NAME[trick]);
  snprintf(line, sizeof line, "%-6s%-7s", st < 8 ? STATE_NAME[st] : "?", extra);
  if (st == ST_WALK) { line[13] = 's'; line[14] = '1' + (speed > 2 ? 2 : speed); }
  else               { line[13] = ' '; line[14] = ' '; }
  line[15] = link ? '*' : ' ';
  line[16] = 0;
  lcdSet(1, line);
}

void startWifi() {
  if (WiFi.status() == WL_NO_MODULE) { Serial.println(F("Wi-Fi module not found")); return; }
  WiFi.config(IPAddress(192, 168, 4, 1));
  if (WiFi.beginAP(AP_SSID, AP_PASS) != WL_AP_LISTENING) { Serial.println(F("could not start Wi-Fi")); return; }
  delay(1500);
  udp.begin(UDP_PORT);
  web.begin();
#if QB_OTA
  ArduinoOTA.begin(WiFi.localIP(), AP_SSID, OTA_PASS, InternalStorage);
#endif
  wifiOk = true;
  Serial.print(F("Wi-Fi network ")); Serial.print(AP_SSID);
  Serial.print(F(" up, robot at ")); Serial.println(WiFi.localIP());
}

void pollUdp(uint32_t now) {
  for (uint8_t k = 0; k < 4; k++) {
    int n = udp.parsePacket();
    if (n <= 0) break;
    uint8_t b[8];
    int len = udp.read(b, sizeof b);
    if (len != 7 || b[0] != 'Q' || b[1] != '4') continue;
    if ((uint8_t)(b[0] ^ b[1] ^ b[2] ^ b[3] ^ b[4] ^ b[5]) != b[6]) continue;
    linkX = (int8_t)b[3]; linkY = (int8_t)b[4]; linkSw = b[5] & 1;
    lastPacket = now; fromPhone = false;
    if ((++replyCount & 3) == 0) {                 // tell the remote what we are doing
      uint8_t r[5] = {'Q', 'R', shownState, shownSpeed, 0};
      r[4] = r[0] ^ r[1] ^ r[2] ^ r[3];
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write(r, sizeof r);
      udp.endPacket();
    }
  }
}

// ---- leg by leg: the page's sliders and the serial 'l' command -----------------
// Leg i (0 FL, 1 FR, 2 RL, 3 RR; 4 = all) to deg, + = foot forward.  The first
// call pauses the gait; legResume() puts the legs vertical and hands control back.
void legManual(int i, int deg) {
  if (i < 0 || i > 4) return;
  if (state != ST_MANUAL) {
    demoOn = false; trickPending = TR_NONE; trick = TR_NONE;
    attachAll(); moving = false; sitPending = false;
    phase[0] = phase[1] = 0; segIdx = 0;
    state = ST_MANUAL;
  }
  deg = constrain(deg, -60, 60);
  if (i == 4) for (uint8_t k = 0; k < 4; k++) writeLeg(k, deg);
  else writeLeg(i, deg);
}

void legResume(uint32_t now) {
  if (state != ST_MANUAL) return;
  float z[4] = {0, 0, 0, 0};
  moveBlocking(z, 400);
  standStill(now);
}

// RESET from any state: stop whatever is going on, every leg to vertical, stand.
void legReset(uint32_t now) {
  if (state == ST_CALIB) { exitCalib(false, now); return; }   // cancels, legs to zero
  serialCmd = CMD_STOP; sitPending = false; wakeRequest = false;
  moving = false; hurry = false;
  demoOn = false; trickPending = TR_NONE; trick = TR_NONE;
  phase[0] = phase[1] = 0; segIdx = 0;
  attachAll();                                   // servos are off while resting
  float z[4] = {0, 0, 0, 0};
  moveBlocking(z, 500);
  waitRelease = true;                            // ignore a button still held
  standStill(now);
}

// A trick from the page or serial: n 1-9.  Not with an empty battery.
void askTrick(uint8_t n) {
  if (battState == BATT_EMPTY) { Serial.println(F("battery empty: no tricks")); return; }
  if (state == ST_MANUAL) legResume(millis());
  requestTrick(n);
}

void pollWeb(uint32_t now) {
  WiFiClient c = web.available();
  if (!c) return;
  char line[72];
  uint8_t n = 0;
  uint32_t t0 = millis();
  while (c.connected() && millis() - t0 < 80) {
    if (!c.available()) continue;
    char ch = c.read();
    if (ch == '\n') break;
    if (n < sizeof line - 1) line[n++] = ch;
  }
  line[n] = 0;
  if (strncmp(line, "GET /j?", 7) == 0) {
    int x = 0, y = 0, s = 0;
    sscanf(line + 7, "x=%d&y=%d&s=%d", &x, &y, &s);
    linkX = (int8_t)constrain(x, -100, 100); linkY = (int8_t)constrain(y, -100, 100); linkSw = s;
    lastPacket = now; fromPhone = true;
    c.print(F("HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n"));
  } else if (strncmp(line, "GET /l?", 7) == 0) {        // leg by leg
    int i = -1, a = 0, x = 0;
    int r = 0;
    if (sscanf(line + 7, "i=%d&a=%d", &i, &a) != 2 && sscanf(line + 7, "x=%d", &x) != 1)
      sscanf(line + 7, "r=%d", &r);
    if (r) legReset(now); else if (x) legResume(now); else legManual(i, a);
    c.print(F("HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n"));
  } else if (strncmp(line, "GET /t?", 7) == 0) {        // tricks: 1-9, 10 demo, 0 stop
    int t = -1;
    sscanf(line + 7, "n=%d", &t);
    if (t == 0) trickStop(now);
    else if (t == 10) { if (battState != BATT_EMPTY) setDemo(!demoOn); }
    else if (t > 0) askTrick((uint8_t)t);
    c.print(F("HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n"));
  } else if (strncmp(line, "GET /o?", 7) == 0) {        // setup: orientation
    if (strncmp(line + 7, "f=1", 3) == 0) swapFrontBack();
    else if (strncmp(line + 7, "m=1", 3) == 0) turnMatrix();
    c.print(F("HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n"));
  } else if (strncmp(line, "GET /b ", 7) == 0) {        // battery text for the page
    char msg[80];
    battMessage(msg, sizeof msg);
    c.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"));
    c.print(msg);
  } else {
    c.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"));
    c.print(PAGE);
  }
  c.stop();
}

bool linkLive(uint32_t now) { return now - lastPacket < (fromPhone ? PHONE_MS : LINK_MS); }

uint8_t inputMask(uint32_t now) {
  if (!linkLive(now) || battState == BATT_EMPTY) return stickMask(0, 0, false);
  return stickMask(linkX, linkY, linkSw);
}

uint8_t inputSpeed() {
  if (!linkLive(millis())) return 0;
  return stickSpeed(linkX, linkY);
}

void inputStatus(uint8_t st, uint8_t cmd, uint8_t speed) {
  shownState = st; shownCmd = cmd; shownSpeed = speed;
  uint32_t now = millis();
  if (now - lastMatrix < 100) return;
  lastMatrix = now;
  bool link = linkLive(now);
  updateLcd(st, cmd, speed, link);
  // battery gauge: EMPTY flashes all the time, LOW flashes 1 s in every 4,
  // otherwise 2 s in every 8 while standing or resting
  uint8_t pct = battPercent();
  if (battState == BATT_EMPTY) { matrixShowBattery(0, link, now % 1000 >= 600); return; }
  if (battState == BATT_LOW && now % 4000 < 1000) { matrixShowBattery(pct, link, now % 500 >= 250); return; }
  if (battState == BATT_OK && (st == ST_STAND || st == ST_REST) && now % 8000 < 2000) {
    matrixShowBattery(pct, link, false); return;
  }
  uint8_t pic = PIC_STOP;
  if (st == ST_REST || st == ST_SIT) pic = PIC_REST;
  else if (st == ST_CALIB || st == ST_MANUAL) pic = PIC_CAL;
  else if (st == ST_TRICK)           pic = trickPic;
  else if (st == ST_WALK)            pic = picForCmd(cmd);
  matrixShow(pic, st == ST_WALK ? speed : 255, link);
}

// Sketch-specific serial commands (the rest are in quadbot_core.h):
//   j          link status
//   l I DEG    leg I (0-3, 4 = all) to DEG, pauses the gait   e.g. "l 1 30"
//   n          back to normal: legs vertical, gait resumes
//   h          RESET from any state: everything stops, legs vertical, stand
//   F          swap front / back (standing or resting only; saved)
//   m          turn the LED pictures 90 deg (saved)
//   b          battery voltage, charge and state
//   B V        calibrate the battery reading to a multimeter's V   e.g. "B 7.95"
//   t N        trick N (1-9); t0 lists them                          e.g. "t4" = bark
//   T          demo: every trick in turn, until T again
void inputSerial(char c) {
  if (c == 't') {
    int n = Serial.parseInt();
    if (n >= 1 && n < TR_COUNT) askTrick((uint8_t)n); else printTricks();
  } else if (c == 'T') {
    if (battState == BATT_EMPTY) Serial.println(F("battery empty: no tricks"));
    else setDemo(!demoOn);
  } else if (c == 'b') {
    battPrint();
  } else if (c == 'B') {
    battCalibrate(Serial.parseFloat());
    battPrint();
  } else if (c == 'F') {
    swapFrontBack();
  } else if (c == 'm') {
    turnMatrix();
  } else if (c == 'h') {
    legReset(millis());
    Serial.println(F("reset: legs vertical"));
  } else if (c == 'l') {
    int i = Serial.parseInt(), deg = Serial.parseInt();
    legManual(i, deg);
    Serial.print(F("leg ")); Serial.print(i); Serial.print(' '); Serial.println(deg);
  } else if (c == 'n') {
    legResume(millis());
    Serial.println(F("walking"));
  } else if (c == 'j') {
    Serial.print(F("link ")); Serial.print(linkLive(millis()) ? F("up") : F("down"));
    Serial.print(fromPhone ? F(" (phone)") : F(" (remote)"));
    Serial.print(F("  x ")); Serial.print(linkX); Serial.print(F("  y ")); Serial.print(linkY);
    Serial.print(F("  pressed ")); Serial.println(linkSw);
  }
}

void setup() {
  loadOrientation();                 // coreSetup loads it again; needed here for the display
  loadMatrixFix();
  ledMatrix.begin();
  matrixShow(PIC_SEARCH, 255, false);
  Serial.begin(115200);
  battBegin();
  if (lcdBegin()) { lcdSet(0, "QuadBot-4"); lcdSet(1, "starting Wi-Fi"); lcdFlush(32); }
  startWifi();
  if (battState != BATT_NONE) { matrixShowBattery(battPercent(), false, false); delay(1200); }
  coreSetup();
  battPrint();
  lcdPrintKind();
  printTricks();
}

void loop() {
  uint32_t now = millis();
  battGuard(now);
  lcdFlush();
  if (wifiOk && now - lastUdpPoll >= 10) { lastUdpPoll = now; pollUdp(now); }
  if (wifiOk && now - lastWebPoll >= 40) { lastWebPoll = now; pollWeb(millis()); }
#if QB_OTA
  if (wifiOk) ArduinoOTA.poll();
#endif
  trickLoop(millis());
  coreLoop();
}
