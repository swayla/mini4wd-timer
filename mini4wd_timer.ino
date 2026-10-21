/*
  四驅車雙道藍芽計時器  Mini4WD Dual-Lane BLE Lap Timer
  硬體：ESP32 30P 開發板 + 擴展板、紅外線對射感測器 x2、TM1637 x2、按鍵 x2
  通訊：BLE Nordic UART Service（NUS），手機/電腦用 Web Bluetooth 網頁連線

  需要安裝的函式庫（Arduino IDE → 管理程式庫）：
    - TM1637 （作者 Avinash Gupta，程式庫名稱 "TM1637"）
  ESP32 開發板套件：Espressif Systems "esp32"（2.x 或 3.x 皆可）
  開發板選擇：ESP32 Dev Module

  計時方式：
    - 預設「過線計時」：每台車第一次過線開始計時，之後每過一次線記一圈，不限圈數。
    - 按「開始」（實體按鍵或網頁）：倒數 3-2-1 後兩台車同時起跑，計時從 GO 開始，
      跑完設定圈數即完賽並排名。完賽或按「重置」後回到過線計時。
    - 支援立體交叉軌道：車子每圈換道，程式會自動判斷每次觸發屬於哪一台車。
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <TM1637Display.h>
#include "esp_timer.h"

#define FW_VERSION  "1.0"
#define DEVICE_NAME "Mini4WD-Timer"

// ───────── 腳位設定 ─────────
const int PIN_SENSOR[2] = {18, 19};   // 感測器白(黃)線：A道、B道
const int PIN_BTN_START = 27;         // 開始按鍵 OUT（VCC 不接）
const int PIN_BTN_RESET = 13;         // 重置按鍵 OUT（VCC 不接）
const int PIN_LED       = 2;          // 板上藍燈：任一道光束被遮斷時亮（對位用）
const int PIN_BUZZER    = -1;         // 有源蜂鳴器接 23 就把 -1 改成 23
const int DISP_CLK[2]   = {25, 32};   // 1號車顯示器、2號車顯示器
const int DISP_DIO[2]   = {26, 33};

// 對射感測器（NPN）遮斷時輸出 LOW。若改用 TSSP4038 方案，改成 HIGH。
#define BEAM_BROKEN_LEVEL LOW

// ───────── BLE NUS UUID ─────────
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // 手機 → ESP32
#define NUS_TX      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // ESP32 → 手機

// ───────── 設定（存在 NVS，斷電保留）─────────
struct Settings {
  uint8_t  raceLaps   = 3;     // 同時起跑的比賽圈數，0 = 不限
  bool     crossover  = true;  // 立體交叉軌道（每圈換道）
  uint8_t  carsMask   = 3;     // bit0 = 1號車（從A道出發）, bit1 = 2號車（從B道出發）
  uint16_t debounceMs = 300;   // 同一道觸發後忽略的時間（避免底盤開孔重複觸發）
  uint16_t ignoreMs   = 1500;  // 同時起跑時，GO 後這段時間內的過線視為起跑線，不算一圈
} cfg;
Preferences prefs;

// ───────── 計時狀態 ─────────
enum State : uint8_t { ST_FREE, ST_COUNTDOWN, ST_RACE, ST_FINISHED };
const char* STATE_NAME[] = {"FREE", "COUNTDOWN", "RACE", "FINISHED"};
State state = ST_FREE;

struct Car {
  bool    active;
  bool    started;       // 已開始計時
  bool    startCrossed;  // 比賽中已通過起跑線
  bool    finished;
  uint8_t place;
  uint16_t laps;
  int64_t startT, lastT; // 微秒
  int64_t bestLap, total;
} cars[2];

int64_t goT = 0;
uint8_t finishCount = 0;
uint32_t cdStartMs = 0;
int cdShown = -1;

// 每道的「下一台會經過的車」佇列（交叉軌道用）
struct LaneQueue { int8_t q[4]; uint8_t n; } laneQ[2];

void qClear(int l) { laneQ[l].n = 0; }
void qPush(int l, int car) { if (laneQ[l].n < 4) laneQ[l].q[laneQ[l].n++] = car; }
int  qPop(int l) {
  if (laneQ[l].n == 0) return -1;
  int c = laneQ[l].q[0];
  for (int i = 1; i < laneQ[l].n; i++) laneQ[l].q[i - 1] = laneQ[l].q[i];
  laneQ[l].n--;
  return c;
}

// ───────── 中斷：只記錄時間戳 ─────────
struct BeamEvent { uint8_t lane; int64_t t; };
#define EVQ 32
volatile BeamEvent evBuf[EVQ];
volatile uint8_t evHead = 0, evTail = 0;
volatile int64_t lastEdge[2] = {0, 0};
volatile int64_t debounceUs = 300000;
portMUX_TYPE evMux = portMUX_INITIALIZER_UNLOCKED;

static inline void IRAM_ATTR onBeam(uint8_t lane) {
  int64_t t = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&evMux);
  if (t - lastEdge[lane] >= debounceUs) {
    lastEdge[lane] = t;
    uint8_t nxt = (evHead + 1) % EVQ;
    if (nxt != evTail) {
      evBuf[evHead].lane = lane;
      evBuf[evHead].t = t;
      evHead = nxt;
    }
  }
  portEXIT_CRITICAL_ISR(&evMux);
}
void IRAM_ATTR isrLaneA() { onBeam(0); }
void IRAM_ATTR isrLaneB() { onBeam(1); }

bool popEvent(BeamEvent& e) {
  bool ok = false;
  portENTER_CRITICAL(&evMux);
  if (evTail != evHead) {
    e.lane = evBuf[evTail].lane;
    e.t = evBuf[evTail].t;
    evTail = (evTail + 1) % EVQ;
    ok = true;
  }
  portEXIT_CRITICAL(&evMux);
  return ok;
}

// ───────── 顯示器 ─────────
TM1637Display disp0(DISP_CLK[0], DISP_DIO[0]);
TM1637Display disp1(DISP_CLK[1], DISP_DIO[1]);
TM1637Display* disp[2] = {&disp0, &disp1};
const uint8_t SEG_DASH[4]  = {SEG_G, SEG_G, SEG_G, SEG_G};
const uint8_t SEG_BLANK[4] = {0, 0, 0, 0};
const uint8_t SEG_GO[4]    = {0, SEG_A | SEG_C | SEG_D | SEG_E | SEG_F, SEG_C | SEG_D | SEG_E | SEG_G, 0}; // " Go "

void showDash(int i)  { disp[i]->setSegments(cars[i].active ? SEG_DASH : SEG_BLANK); }

// 小於 100 秒顯示「秒:百分秒」，例如 5:23 = 5.23 秒；100 秒以上顯示「分:秒」
void showTime(int i, int64_t us) {
  uint32_t cs = (uint32_t)(us / 10000);
  uint32_t a, b;
  if (cs < 10000) { a = cs / 100; b = cs % 100; }
  else { uint32_t s = cs / 100; a = s / 60; b = s % 60; if (a > 99) { showDash(i); return; } }
  uint8_t seg[4];
  seg[0] = a >= 10 ? disp[i]->encodeDigit(a / 10) : 0;
  seg[1] = disp[i]->encodeDigit(a % 10) | 0x80;   // 0x80 = 中間冒號
  seg[2] = disp[i]->encodeDigit(b / 10);
  seg[3] = disp[i]->encodeDigit(b % 10);
  disp[i]->setSegments(seg);
}

void showCount(int i, int n) {
  uint8_t seg[4] = {0, 0, 0, 0};
  seg[1] = disp[i]->encodeDigit(n);
  disp[i]->setSegments(cars[i].active ? seg : SEG_BLANK);
}

// ───────── 蜂鳴器（非阻塞）─────────
uint32_t buzzOffAt = 0;
void beep(uint16_t ms) {
  if (PIN_BUZZER < 0) return;
  digitalWrite(PIN_BUZZER, HIGH);
  buzzOffAt = millis() + ms;
}

// ───────── BLE ─────────
BLEServer* bleServer = nullptr;
BLECharacteristic* txChar = nullptr;
volatile bool bleConnected = false;
volatile bool needAdvertise = false;
QueueHandle_t cmdQueue;
#define CMD_LEN 48

void sendLine(const char* line) {
  Serial.println(line);
  if (!bleConnected || !txChar) return;
  String s = String(line) + "\n";
  const uint8_t* p = (const uint8_t*)s.c_str();
  size_t n = s.length();
  for (size_t o = 0; o < n; o += 20) {        // 分段送出，相容預設 MTU
    size_t k = (n - o) < 20 ? (n - o) : 20;
    txChar->setValue((uint8_t*)(p + o), k);
    txChar->notify();
    delay(4);
  }
}

void sendf(const char* fmt, ...) {
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  sendLine(buf);
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConnected = true; }
  void onDisconnect(BLEServer*) override { bleConnected = false; needAdvertise = true; }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = String(c->getValue().c_str());
    int start = 0;
    while (start < (int)v.length()) {
      int nl = v.indexOf('\n', start);
      if (nl < 0) nl = v.length();
      String line = v.substring(start, nl);
      line.trim();
      if (line.length()) {
        char buf[CMD_LEN];
        line.toCharArray(buf, CMD_LEN);
        xQueueSend(cmdQueue, buf, 0);
      }
      start = nl + 1;
    }
  }
};

// ───────── 設定存取 ─────────
void loadSettings() {
  prefs.begin("m4wd", true);
  cfg.raceLaps   = prefs.getUChar("laps", 3);
  cfg.crossover  = prefs.getBool("cross", true);
  cfg.carsMask   = prefs.getUChar("cars", 3);
  cfg.debounceMs = prefs.getUShort("deb", 300);
  cfg.ignoreMs   = prefs.getUShort("ign", 1500);
  prefs.end();
  if (cfg.carsMask < 1 || cfg.carsMask > 3) cfg.carsMask = 3;
  debounceUs = (int64_t)cfg.debounceMs * 1000;
}

void saveSettings() {
  prefs.begin("m4wd", false);
  prefs.putUChar("laps", cfg.raceLaps);
  prefs.putBool("cross", cfg.crossover);
  prefs.putUChar("cars", cfg.carsMask);
  prefs.putUShort("deb", cfg.debounceMs);
  prefs.putUShort("ign", cfg.ignoreMs);
  prefs.end();
  debounceUs = (int64_t)cfg.debounceMs * 1000;
}

void sendConfig() {
  sendf("CFG,%u,%u,%u,%u,%u", cfg.raceLaps, cfg.crossover ? 1 : 0, cfg.carsMask, cfg.debounceMs, cfg.ignoreMs);
}

// ───────── 計時邏輯 ─────────
void resetCars() {
  for (int i = 0; i < 2; i++) {
    Car& c = cars[i];
    c.active = cfg.carsMask & (1 << i);
    c.started = c.startCrossed = c.finished = false;
    c.place = 0; c.laps = 0;
    c.startT = c.lastT = 0;
    c.bestLap = 0; c.total = 0;
  }
  finishCount = 0;
  // 1號車從A道、2號車從B道出發
  qClear(0); qClear(1);
  if (cars[0].active) qPush(0, 0);
  if (cars[1].active) qPush(1, 1);
}

void setState(State s) {
  state = s;
  sendf("ST,%s", STATE_NAME[s]);
}

void resetAll() {
  resetCars();
  for (int i = 0; i < 2; i++) showDash(i);
  setState(ST_FREE);
  sendLine("RST");
}

void startCountdown() {
  resetCars();
  cdStartMs = millis();
  cdShown = -1;
  setState(ST_COUNTDOWN);
}

void sendCrossing(int car, int lane, uint16_t lap, int64_t lapUs, int64_t totalUs, int64_t t) {
  sendf("X,%d,%c,%u,%lld,%lld,%lld", car + 1, lane ? 'B' : 'A', lap,
        (long long)lapUs, (long long)totalUs, (long long)t);
}

void countLap(int car, int lane, int64_t t) {
  Car& c = cars[car];
  int64_t lap = t - c.lastT;
  c.lastT = t;
  c.laps++;
  c.total = t - c.startT;
  if (c.bestLap == 0 || lap < c.bestLap) c.bestLap = lap;
  sendCrossing(car, lane, c.laps, lap, c.total, t);
  showTime(car, lap);
  beep(60);
}

void handleCrossing(int lane, int64_t t) {
  if (state == ST_COUNTDOWN) { sendf("FS,%c", lane ? 'B' : 'A'); return; }  // 偷跑，不計
  if (state == ST_FINISHED) return;

  int car = qPop(lane);
  if (car < 0) { sendf("U,%c", lane ? 'B' : 'A'); return; }               // 這道沒有預期的車
  qPush(cfg.crossover ? 1 - lane : lane, car);

  Car& c = cars[car];
  if (state == ST_FREE) {
    if (!c.started) {
      c.started = true;
      c.startT = c.lastT = t;
      sendCrossing(car, lane, 0, 0, 0, t);
      showTime(car, 0);
      beep(60);
    } else {
      countLap(car, lane, t);
    }
    return;
  }

  // ST_RACE
  if (c.finished) return;
  if (!c.startCrossed && !c.laps && (t - goT) < (int64_t)cfg.ignoreMs * 1000) {
    c.startCrossed = true;                       // 起跑線，不算一圈
    sendCrossing(car, lane, 0, 0, t - goT, t);
    return;
  }
  countLap(car, lane, t);
  if (cfg.raceLaps && c.laps >= cfg.raceLaps) {
    c.finished = true;
    c.place = ++finishCount;
    sendf("FIN,%d,%lld,%u", car + 1, (long long)c.total, c.place);
    showTime(car, c.total);
    beep(400);
    bool all = true;
    for (int i = 0; i < 2; i++) if (cars[i].active && !cars[i].finished) all = false;
    if (all) { setState(ST_FINISHED); sendLine("END"); }
  }
}

void updateCountdown() {
  uint32_t el = millis() - cdStartMs;
  int step = el / 1000;                 // 0,1,2 → 3,2,1；3 → GO
  if (step == cdShown) return;
  cdShown = step;
  if (step < 3) {
    for (int i = 0; i < 2; i++) showCount(i, 3 - step);
    sendf("CD,%d", 3 - step);
    beep(120);
  } else {
    goT = esp_timer_get_time();
    for (int i = 0; i < 2; i++) {
      cars[i].started = cars[i].active;
      cars[i].startT = cars[i].lastT = goT;
      disp[i]->setSegments(cars[i].active ? SEG_GO : SEG_BLANK);
    }
    setState(ST_RACE);
    sendf("GO,%lld", (long long)goT);
    beep(600);
  }
}

// ───────── 指令處理 ─────────
void sendStatus() {
  sendf("HELLO,%s", FW_VERSION);
  sendConfig();
  sendf("ST,%s", STATE_NAME[state]);
  for (int i = 0; i < 2; i++) {
    Car& c = cars[i];
    sendf("C,%d,%d,%d,%u,%lld,%lld,%d,%u", i + 1, c.active, c.started, c.laps,
          (long long)c.bestLap, (long long)c.total, c.finished, c.place);
  }
}

void handleCommand(char* cmd) {
  char* arg = strchr(cmd, ',');
  if (arg) *arg++ = 0;
  int v = arg ? atoi(arg) : 0;

  if      (!strcmp(cmd, "START"))  startCountdown();
  else if (!strcmp(cmd, "RESET"))  resetAll();
  else if (!strcmp(cmd, "STATUS")) sendStatus();
  else if (!strcmp(cmd, "LAPS"))   { cfg.raceLaps = constrain(v, 0, 99); saveSettings(); sendConfig(); }
  else if (!strcmp(cmd, "CROSS"))  { cfg.crossover = v != 0; saveSettings(); sendConfig(); resetAll(); }
  else if (!strcmp(cmd, "CARS"))   { cfg.carsMask = constrain(v, 1, 3); saveSettings(); sendConfig(); resetAll(); }
  else if (!strcmp(cmd, "DEB"))    { cfg.debounceMs = constrain(v, 50, 3000); saveSettings(); sendConfig(); }
  else if (!strcmp(cmd, "IGN"))    { cfg.ignoreMs = constrain(v, 0, 10000); saveSettings(); sendConfig(); }
  else sendf("ERR,%s", cmd);
}

// ───────── 按鍵 ─────────
struct Button { int pin; bool last; uint32_t changedAt; bool stable; };
Button btnStart = {PIN_BTN_START, HIGH, 0, HIGH};
Button btnReset = {PIN_BTN_RESET, HIGH, 0, HIGH};

bool pressed(Button& b) {           // 按下瞬間回傳 true（含 30ms 去彈跳）
  bool r = digitalRead(b.pin);
  if (r != b.last) { b.last = r; b.changedAt = millis(); }
  if (millis() - b.changedAt > 30 && r != b.stable) {
    b.stable = r;
    if (r == LOW) return true;
  }
  return false;
}

// ───────── 光束狀態（對位用）─────────
bool beamOk[2] = {true, true};
uint32_t blockedSince[2] = {0, 0};
uint32_t lastSensPoll = 0;

void pollBeams() {
  if (millis() - lastSensPoll < 50) return;
  lastSensPoll = millis();
  bool anyBlocked = false;
  bool changed = false;
  for (int l = 0; l < 2; l++) {
    bool blocked = digitalRead(PIN_SENSOR[l]) == BEAM_BROKEN_LEVEL;
    anyBlocked |= blocked;
    if (blocked) { if (!blockedSince[l]) blockedSince[l] = millis(); }
    else blockedSince[l] = 0;
    bool ok = !(blocked && millis() - blockedSince[l] > 1000);   // 持續遮斷 1 秒才算異常
    if (ok != beamOk[l]) { beamOk[l] = ok; changed = true; }
  }
  digitalWrite(PIN_LED, anyBlocked ? HIGH : LOW);
  if (changed) sendf("SENS,%d,%d", beamOk[0], beamOk[1]);
}

// ───────── setup / loop ─────────
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  if (PIN_BUZZER >= 0) { pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW); }
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_RESET, INPUT_PULLUP);
  for (int l = 0; l < 2; l++) pinMode(PIN_SENSOR[l], INPUT_PULLUP);

  loadSettings();

  for (int i = 0; i < 2; i++) { disp[i]->setBrightness(7); disp[i]->clear(); }
  resetCars();
  for (int i = 0; i < 2; i++) showDash(i);

  int edge = (BEAM_BROKEN_LEVEL == LOW) ? FALLING : RISING;
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR[0]), isrLaneA, edge);
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR[1]), isrLaneB, edge);

  cmdQueue = xQueueCreate(8, CMD_LEN);

  BLEDevice::init(DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCB());
  BLEService* svc = bleServer->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  BLECharacteristic* rx = svc->createCharacteristic(
      NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("Mini4WD timer ready " FW_VERSION);
}

void loop() {
  BeamEvent e;
  while (popEvent(e)) handleCrossing(e.lane, e.t);

  if (state == ST_COUNTDOWN) updateCountdown();

  char cmd[CMD_LEN];
  while (xQueueReceive(cmdQueue, cmd, 0) == pdTRUE) handleCommand(cmd);

  if (pressed(btnStart)) startCountdown();
  if (pressed(btnReset)) resetAll();

  pollBeams();

  if (buzzOffAt && (int32_t)(millis() - buzzOffAt) >= 0) { digitalWrite(PIN_BUZZER, LOW); buzzOffAt = 0; }

  if (needAdvertise) {
    needAdvertise = false;
    delay(200);
    BLEDevice::startAdvertising();
  }
}
