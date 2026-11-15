/*
 * BLUE-WARNER – Freenove ESP32 Display 3.5" (ST7796, 320x480, resistive touch, LVGL 8.4)
 * Developed by A. Isaksson, SA7BNB
 *
 * Passiv Wi-Fi-scanner (promiscuous mode, kanal 1-13) som larmar när en MAC med
 * bevakad OUI (tre första byten) hörs.
 *
 * + FARTKAMERAVARNARE (sammanslagen med Speed Camera Detector v2.5):
 *   GPS (NMEA, 9600 baud) läses på GPS_RX_PIN. Inom CAM_RADIUS_M från en kamera visas
 *   "SPEED CAMERA" med gällande gräns, och om farten överstiger gräns + SPEED_TOLERANCE
 *   visas "SLOW DOWN" med pip (3 kHz, tätare ju större överträdelse). Wi-Fi-larmet har
 *   en egen ton (2 kHz, pulserande) så att de två går att skilja åt på ljudet.
 *
 *   GUL   "CONFIG MODE"  – första 30 s efter start: Wi-Fi-AP "BLUE" (lösen 123456789) + webbsida
 *                       på 192.168.4.1 för att lägga till/ta bort OUI. Ansluter någon hålls AP:n
 *                       uppe tills klienten kopplar från, därefter startar skanningen.
 *   GRÖN  "ALL OK"      – skanning pågår, inget bevakat hörs
 *   RÖD   "ALARM"       – bevakad enhet i närheten (MAC + RSSI visas) + ljud i högtalaren
 *   BLÅ   "LEARN MODE"  – 30 s, de 10 starkaste MAC sparas (OUI läggs till i listan)
 *   GUL   "SPEED CAMERA" – fartkamera inom 500 m, farten är inom gränsen
 *   ORANGE "SLOW DOWN"  – fartkamera inom 500 m och farten är över gränsen
 *   Prioritet på skärmen: CONFIG > LEARN > SLOW DOWN > ALARM > SPEED CAMERA > ALL OK
 *
 * LISTAN LIGGER ALLTID I RAM: vid uppstart läses /oui.txt från SD en gång in i RAM och speglas
 * till flash (NVS, EEPROM-ersättning). Under drift rör skanning och UI aldrig SD-kortet; all
 * SD-skrivning (logg, lärda MAC, sparad lista) sker i en egen bakgrundstråd.
 * Prioritet vid start: 1) /oui.txt på SD, 2) kopian i flash, 3) standardlistan.
 *
 * SD-kort (valfritt):
 *   /oui.txt          bevakade OUI, en per rad (AA:BB:CC). Kan redigeras på dator.
 *   /oui_log.csv      händelselogg (uppstartstid, typ, MAC, RSSI)
 *   /oui_learned.csv  fullständiga MAC som lärts in
 *   /cameras.csv      fartkameror, en per rad: lat,lon,gräns   (57.399991,14.892603,80)
 *   /speed_cameras_with_limits.h   går lika bra: kopiera .h-filen som den är till kortet
 *                     (rader på formen {57.399991, 14.892603, 80}, läses också).
 *   Kameralistan läses in i RAM en gång vid start. Finns ingen fil på kortet används
 *   listan som kompilerats in från speed_cameras_with_limits.h i sketchmappen.
 *
 * INSTALLATION
 *   1. Kopiera Freenoves mapp "Sketch_13.1_LVGL" (innehåller display.h m.fl. och lv_conf)
 *      och byt ut .ino-filen mot denna (mappen och .ino ska heta samma sak).
 *   2. Bibliotek: de som följer med Freenove (LVGL v8.4.0, TFT_eSPI). Uppdatera inte dem.
 *   3. Board: ESP32 Dev Module, Partition Scheme: "Huge APP (3MB No OTA)".
 *   4. Lägg speed_cameras_with_limits.h i samma mapp som .ino-filen (inbyggd reservlista).
 *   5. GPS: VCC -> 3.3V, GND -> GND, GPS TX -> GPIO35 (GPS_RX_PIN). GPS RX ansluts inte.
 *
 * ANTAGANDEN (kontrollera mot ditt kort):
 *   - SD-kortet sitter på VSPI (GPIO5/18/19/23) och TFT på HSPI (13/12/14), enligt Freenove.
 *   - Högtalarförstärkaren matas från GPIO26 (ESP32 DAC2). Ljud som uteblir -> prova 25.
 *   - RGB-LED är aktiv låg (vanligt på CYD). Ändra RGB_ACTIVE_LOW om färgerna är inverterade.
 */

#include <WiFi.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <WebServer.h>
#include "esp_wifi.h"
#include "display.h"          // Freenoves LVGL-wrapper (screen.init() / screen.routine())

// ---------- GPS / fartkameror ----------
#define GPS_RX_PIN      35        // GPIO35 = kortets RX, hit ansluts GPS-modulens TX (endast mottagning)
#define GPS_BAUD        9600
#define GPS_DEBUG       2         // seriemonitor: 0 = av, 1 = statusrad var 2:a sekund, 2 = även råa NMEA-rader
#define GPS_TIMEOUT_MS  5000      // ingen giltig position så här länge -> fix förlorad
#define CAM_RADIUS_M    500       // varna inom detta avstånd
#define SPEED_TOLERANCE 1         // km/h över gränsen innan det piper
#define SPEED_FREQ      3000      // ton för fartvarning (Wi-Fi-larmet använder BEEP_FREQ)
#define CAM_FILE_CSV    "/cameras.csv"
#define CAM_FILE_H      "/speed_cameras_with_limits.h"
#define BUILTIN_CAMERAS 1         // 1 = kompilera in .h-filen som reservlista, 0 = endast SD

#if BUILTIN_CAMERAS
  #include "speed_cameras_with_limits.h"   // struct SpeedCamera, speedCameras[], numCameras
#else
  struct SpeedCamera { float latitude; float longitude; int speedLimit; };
#endif

// ---------- Pinnar ----------
#define SD_CS         5
#define SD_SCK        18
#define SD_MISO       19
#define SD_MOSI       23
#define SPK_PIN       26          // DAC2 (verifierat: Freenove använder inbyggd DAC)
#define SPK_PIN2      25          // DAC1, drivs parallellt
#define AUDIO_EN      4           // förstärkaren aktiveras med LOW (Freenove Sketch_07.1)
#define RGB_R         22
#define RGB_G         16
#define RGB_B         17
#define RGB_ACTIVE_LOW 1

// ---------- Inställningar ----------
#define CH_FIRST      1
#define CH_LAST       13
#define DWELL_MS      150
#define HOLD_MS       6000
#define RSSI_MIN      -85
#define LEARN_MS      30000
#define LEARN_MAX     10
#define MAX_OUI       64
#define BEEP_FREQ     2000

#define OUI_FILE      "/oui.txt"
#define LOG_FILE      "/oui_log.csv"
#define LEARN_FILE    "/oui_learned.csv"

#define AP_SSID       "BLUE"
#define AP_PASS       "123456789"
#define AP_WAIT_MS    30000       // väntetid på klient efter start (samma skala som LEARN_MS)
#define AP_DROP_MS    2000        // klient måste ha varit borta så här länge

// ---------- Färger ----------
#define COL_BG     lv_color_hex(0x0B1220)
#define COL_PANEL  lv_color_hex(0x131C2E)
#define COL_LINE   lv_color_hex(0x25324A)
#define COL_TEXT   lv_color_hex(0xE6EBF3)
#define COL_MUTED  lv_color_hex(0x8795AD)
#define COL_RED    lv_color_hex(0xFF4D4F)
#define COL_GREEN  lv_color_hex(0x3DDC84)
#define COL_BLUE   lv_color_hex(0x4DA3FF)
#define COL_AMBER  lv_color_hex(0xFFB020)
#define COL_ORANGE lv_color_hex(0xFF7A1A)

// ---------- Typsnitt (använder största som är aktiverat i lv_conf.h) ----------
#if LV_FONT_MONTSERRAT_48
  #define FONT_ICON (&lv_font_montserrat_48)
#elif LV_FONT_MONTSERRAT_32
  #define FONT_ICON (&lv_font_montserrat_32)
#else
  #define FONT_ICON (&lv_font_montserrat_14)
#endif
#if LV_FONT_MONTSERRAT_32
  #define FONT_XL (&lv_font_montserrat_32)
#elif LV_FONT_MONTSERRAT_28
  #define FONT_XL (&lv_font_montserrat_28)
#elif LV_FONT_MONTSERRAT_24
  #define FONT_XL (&lv_font_montserrat_24)
#else
  #define FONT_XL (&lv_font_montserrat_14)
#endif
#if LV_FONT_MONTSERRAT_20
  #define FONT_L (&lv_font_montserrat_20)
#elif LV_FONT_MONTSERRAT_18
  #define FONT_L (&lv_font_montserrat_18)
#elif LV_FONT_MONTSERRAT_16
  #define FONT_L (&lv_font_montserrat_16)
#else
  #define FONT_L (&lv_font_montserrat_14)
#endif
#define FONT_M (&lv_font_montserrat_14)

// ---------- Standard-OUI ----------
const uint8_t DEFAULT_OUIS[][3] = {
  {0x00, 0xA0, 0xD5}, {0x28, 0xA3, 0x31}, {0x50, 0x13, 0x9D}, {0x58, 0xFC, 0xC8},
  {0x64, 0xCE, 0x6E}, {0x84, 0xDB, 0x2F}, {0xCC, 0x93, 0x4A}, {0x0C, 0xFE, 0x5D},
  {0x00, 0x1E, 0x42}, {0x20, 0x97, 0x27}, {0x00, 0x07, 0x7C}, {0x00, 0x11, 0xB4},
  {0xB4, 0x56, 0xF3}, {0x00, 0x21, 0x7E}, {0x00, 0x80, 0x25}, {0x48, 0x02, 0xAF},
  {0x9C, 0xBA, 0xC9},
};

// ---------- Tillstånd ----------
Display screen;

uint8_t ouiTable[MAX_OUI][3];
uint8_t ouiCount = 0;

volatile uint32_t lastSeen = 0;
volatile int8_t   lastRssi = 0;
volatile uint8_t  lastMac[6];
volatile uint8_t  curCh = CH_FIRST;

struct LearnEntry { uint8_t mac[6]; int8_t rssi; };
LearnEntry learnBuf[LEARN_MAX];
volatile uint8_t learnCount = 0;
volatile bool    learning = false;
uint32_t         learnStart = 0;
portMUX_TYPE     mux = portMUX_INITIALIZER_UNLOCKED;

bool     sdOk = false;
bool     soundOn = true;
bool     nightMode = false;           // nattläge: låg bakgrundsbelysning, mörkare bild, LED släckt (utom larm)
uint32_t beepUntil = 0;
String   logText;
bool     logDirty = true;
bool     listDirty = true;

bool     apMode = true;
bool     clientSeen = false;
uint32_t apStart = 0, lastClientMs = 0;
WebServer server(80);

enum UiState { ST_NONE = -1, ST_OK = 0, ST_ALARM = 1, ST_LEARN = 2, ST_AP = 3, ST_CAM = 4, ST_SPEED = 5 };
int uiState = ST_NONE;

// GPS + fartkameror (allt detta körs i loop()-tråden, samma som LVGL)
HardwareSerial GPS(2);
const SpeedCamera *cams = nullptr;    // pekar på RAM-kopian från SD eller på den inbyggda listan
int      camCount = 0;
bool     camsFromSd = false;
bool     gpsData = false, gpsFix = false;
uint32_t lastGpsCharMs = 0, lastFixMs = 0;
uint8_t  gpsSats = 0;
uint32_t gpsBytes = 0, gpsOkCnt = 0, gpsBadCnt = 0;   // för GPS_DEBUG
float    curLat = 0, curLon = 0, curSpeed = 0;      // curSpeed i km/h
bool     camNear = false, speeding = false;
int      camIdx = -1, camLimit = 0;
float    camDist = 0;
uint32_t speedBeepUntil = 0, lastSpeedBeep = 0;

// UI-objekt
lv_obj_t *lblHdrRight, *panel, *lblIcon, *lblStatus, *lblNote, *lblSub, *barLearn;
lv_obj_t *btnNightLbl, *nightOv = nullptr;
lv_obj_t *btnLearnLbl, *btnSoundLbl, *listCont, *lblOuiCount, *lblLog;

// ======================= Hjälpfunktioner =======================
static void macStr(const uint8_t *m, char *o) {
  snprintf(o, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

static void rgb(bool r, bool g, bool b) {
  if (nightMode && !(r && !g && !b)) { r = g = b = 0; }   // natt: bara larm (rött) lyser
  digitalWrite(RGB_R, RGB_ACTIVE_LOW ? !r : r);
  digitalWrite(RGB_G, RGB_ACTIVE_LOW ? !g : g);
  digitalWrite(RGB_B, RGB_ACTIVE_LOW ? !b : b);
}

// Bakgrundsbelysning (GPIO27) via PWM
#define BL_PIN        27
#define BL_DAY        255
#define BL_NIGHT      12                              // ~5 %, höj om det blir för mörkt
static void blInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BL_PIN, 5000, 8);
#else
  ledcSetup(3, 5000, 8);
  ledcAttachPin(BL_PIN, 3);
#endif
}
static void blSet(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(BL_PIN, duty);
#else
  ledcWrite(3, duty);
#endif
}

// Högtalare via LEDC (fyrkant) på GPIO26
static void spkInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(SPK_PIN, BEEP_FREQ, 8);
  ledcAttach(SPK_PIN2, BEEP_FREQ, 8);
#else
  ledcSetup(0, BEEP_FREQ, 8);
  ledcAttachPin(SPK_PIN, 0);
  ledcSetup(1, BEEP_FREQ, 8);
  ledcAttachPin(SPK_PIN2, 1);
#endif
}
static void spk(uint16_t freq) {                       // 0 = tyst
  static uint16_t cur = 0;
  if (freq == cur) return;
  cur = freq;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(SPK_PIN, freq);
  ledcWriteTone(SPK_PIN2, freq);
#else
  ledcWriteTone(0, freq);
  ledcWriteTone(1, freq);
#endif
}
static void beepMs(uint16_t ms) { beepUntil = millis() + ms; }

// ======================= OUI-lista + SD =======================
static bool hasOui(const uint8_t *o) {
  for (uint8_t i = 0; i < ouiCount; i++)
    if (ouiTable[i][0] == o[0] && ouiTable[i][1] == o[1] && ouiTable[i][2] == o[2]) return true;
  return false;
}

static bool addOui(const uint8_t *o) {
  if (ouiCount >= MAX_OUI || hasOui(o)) return false;
  for (int j = 0; j < 3; j++) ouiTable[ouiCount][j] = o[j];
  ouiCount++;
  return true;
}

static void removeOui(int idx) {
  if (idx < 0 || idx >= ouiCount) return;
  for (int k = idx; k < ouiCount - 1; k++)
    for (int j = 0; j < 3; j++) ouiTable[k][j] = ouiTable[k + 1][j];
  ouiCount--;
}

static void setDefaults() {
  ouiCount = 0;
  for (size_t i = 0; i < sizeof(DEFAULT_OUIS) / sizeof(DEFAULT_OUIS[0]); i++) addOui(DEFAULT_OUIS[i]);
}

// ---- SD-skrivning i bakgrundstråd (blockerar aldrig UI eller skanning) ----
#define JOB_LOG        1
#define JOB_LEARN      2
#define JOB_SAVE_OUIS  3
struct SdJob { uint8_t kind; char text[96]; };
QueueHandle_t sdQ = nullptr;
Preferences prefs;

static void sdEnqueue(uint8_t kind, const char *text) {
  if (!sdOk || !sdQ) return;
  SdJob j;
  j.kind = kind;
  strlcpy(j.text, text ? text : "", sizeof(j.text));
  xQueueSend(sdQ, &j, 0);                           // släpp jobbet om kön är full, vänta aldrig
}

static void sdTask(void *) {
  SdJob j;
  for (;;) {
    if (xQueueReceive(sdQ, &j, portMAX_DELAY) != pdTRUE) continue;
    if (j.kind == JOB_LOG || j.kind == JOB_LEARN) {
      File f = SD.open(j.kind == JOB_LOG ? LOG_FILE : LEARN_FILE, FILE_APPEND);
      if (f) { f.print(j.text); f.close(); }
    } else if (j.kind == JOB_SAVE_OUIS) {
      uint8_t snap[MAX_OUI][3];
      uint8_t n = ouiCount;
      memcpy(snap, ouiTable, sizeof(snap));
      File f = SD.open(OUI_FILE, FILE_WRITE);
      if (f) {
        for (uint8_t i = 0; i < n; i++) f.printf("%02X:%02X:%02X\n", snap[i][0], snap[i][1], snap[i][2]);
        f.close();
      }
    }
  }
}

// ---- Kopia i flash (NVS) ----
static bool nvsMatches() {
  if (!prefs.isKey("cnt") || prefs.getUChar("cnt", 255) != ouiCount) return false;
  if (ouiCount == 0) return true;
  uint8_t tmp[MAX_OUI * 3];
  size_t len = prefs.getBytes("list", tmp, sizeof(tmp));
  return len == (size_t)ouiCount * 3 && memcmp(tmp, ouiTable, len) == 0;
}

static void saveNvs() {
  if (nvsMatches()) return;                         // spara flash-slitage
  prefs.putUChar("cnt", ouiCount);
  if (ouiCount) prefs.putBytes("list", ouiTable, ouiCount * 3);
}

static void saveOuis() {
  saveNvs();                                        // flash: direkt (några ms)
  sdEnqueue(JOB_SAVE_OUIS, nullptr);                // SD: i bakgrunden
}

// Tolkar en rad: tar de sex första hex-tecknen (AA:BB:CC, AA-BB-CC, AABBCC, full MAC ...)
static void parseOuiLine(const String &line) {
  if (line.length() == 0 || line[0] == '#') return;
  String hex = "";
  for (size_t i = 0; i < line.length() && hex.length() < 6; i++)
    if (isxdigit(line[i])) hex += line[i];
  if (hex.length() < 6) return;
  uint8_t o[3];
  for (int i = 0; i < 3; i++) o[i] = (uint8_t)strtol(hex.substring(i * 2, i * 2 + 2).c_str(), NULL, 16);
  addOui(o);
}

static void loadOuis() {
  ouiCount = 0;

  // 1) SD-kortet vinner: läs hela filen till RAM och spegla till flash
  if (sdOk && SD.exists(OUI_FILE)) {
    File f = SD.open(OUI_FILE, FILE_READ);
    if (f) {
      while (f.available() && ouiCount < MAX_OUI) parseOuiLine(f.readStringUntil('\n'));
      f.close();
      saveNvs();
      return;
    }
  }

  // 2) Kopian i flash (fungerar även utan SD-kort)
  if (prefs.isKey("cnt")) {
    uint8_t n = prefs.getUChar("cnt", 0);
    if (n > MAX_OUI) n = MAX_OUI;
    if (n) prefs.getBytes("list", ouiTable, n * 3);
    ouiCount = n;
    if (sdOk) sdEnqueue(JOB_SAVE_OUIS, nullptr);    // skapa /oui.txt på kortet
    return;
  }

  // 3) Standardlistan
  setDefaults();
  saveOuis();
}

// ======================= Logg =======================
static void addLog(const char *type, const uint8_t *mac, int rssi) {
  uint32_t s = millis() / 1000;
  char m[18] = "";
  if (mac) macStr(mac, m);

  char line[80];
  if (mac) snprintf(line, sizeof(line), "T+%02lu:%02lu:%02lu  %s  %s  %d dBm",
                    (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60), type, m, rssi);
  else     snprintf(line, sizeof(line), "T+%02lu:%02lu:%02lu  %s",
                    (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60), type);

  logText = String(line) + "\n" + logText;
  if (logText.length() > 1400) logText = logText.substring(0, 1400);
  logDirty = true;

  char csv[96];
  snprintf(csv, sizeof(csv), "%lu,%s,%s,%d\n", (unsigned long)s, type, m, rssi);
  sdEnqueue(JOB_LOG, csv);
  Serial.println(line);
}

// ======================= Fartkameror =======================
// Tolkar "57.399991,14.892603,80" och "{57.399991, 14.892603, 80}," (C-headerns format).
// Allt annat (kommentarer, struct-definitionen, numCameras ...) ignoreras.
static bool parseCamLine(const char *s, float *lat, float *lon, int *lim) {
  while (*s == ' ' || *s == '\t' || *s == '{') s++;
  if (!isdigit((unsigned char)*s) && *s != '-') return false;
  char *e;
  double la = strtod(s, &e);
  if (e == s) return false;
  s = e;
  while (*s == ' ' || *s == '\t' || *s == ',' || *s == ';') s++;
  double lo = strtod(s, &e);
  if (e == s) return false;
  s = e;
  while (*s == ' ' || *s == '\t' || *s == ',' || *s == ';') s++;
  long l = strtol(s, &e, 10);
  if (e == s) return false;
  if (la < -90 || la > 90 || lo < -180 || lo > 180 || (la == 0 && lo == 0)) return false;
  if (l < 5 || l > 200) return false;
  *lat = (float)la; *lon = (float)lo; *lim = (int)l;
  return true;
}

static bool readLine(File &f, char *buf, size_t n) {
  size_t i = 0;
  bool any = false;
  while (f.available()) {
    int c = f.read();
    any = true;
    if (c == '\n') break;
    if (c != '\r' && i < n - 1) buf[i++] = (char)c;
  }
  buf[i] = 0;
  return any;
}

// Läser kamerafilen från SD till RAM (två pass: räkna, sedan fyll). true om minst en kamera lästes.
static bool loadCamerasFromSd(const char *path) {
  if (!sdOk || !SD.exists(path)) return false;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  char line[96];
  float la, lo;
  int li, n = 0;
  while (readLine(f, line, sizeof(line)))
    if (parseCamLine(line, &la, &lo, &li)) n++;
  if (n == 0) { f.close(); return false; }

  SpeedCamera *buf = (SpeedCamera *)malloc((size_t)n * sizeof(SpeedCamera));
  if (!buf) { f.close(); Serial.println("Cameras: out of memory"); return false; }
  f.seek(0);
  int k = 0;
  while (k < n && readLine(f, line, sizeof(line)))
    if (parseCamLine(line, &la, &lo, &li)) { buf[k].latitude = la; buf[k].longitude = lo; buf[k].speedLimit = li; k++; }
  f.close();
  cams = buf;
  camCount = k;
  camsFromSd = true;
  return true;
}

static void loadCameras() {
  if (loadCamerasFromSd(CAM_FILE_CSV) || loadCamerasFromSd(CAM_FILE_H)) return;
#if BUILTIN_CAMERAS
  cams = speedCameras;                               // ligger i flash, tar inget RAM
  camCount = numCameras;
#endif
}

// Körs vid varje ny GPS-position (1 Hz). Plan-approximation räcker gott på 500 m och är
// mycket snabbare än haversine över ett par tusen kameror.
static void checkCameras() {
  const float M_PER_DEG = 111195.0f;
  const float win = (CAM_RADIUS_M * 1.2f) / M_PER_DEG;      // grovfilter i grader
  float cosLat = cosf(curLat * 0.01745329252f);
  float best = CAM_RADIUS_M + 1.0f;
  int bi = -1;
  for (int i = 0; i < camCount; i++) {
    float dLat = cams[i].latitude - curLat;
    if (dLat > win || dLat < -win) continue;
    float dLon = (cams[i].longitude - curLon) * cosLat;
    if (dLon > win || dLon < -win) continue;
    float d = sqrtf(dLat * dLat + dLon * dLon) * M_PER_DEG;
    if (d <= CAM_RADIUS_M && d < best) { best = d; bi = i; }
  }

  char msg[48];
  if (bi >= 0) {
    if (!camNear || bi != camIdx) {
      snprintf(msg, sizeof(msg), "CAMERA limit=%d dist=%dm", cams[bi].speedLimit, (int)best);
      addLog(msg, nullptr, 0);
      speeding = false;                              // ny kamera -> logga ev. överträdelse på nytt
    }
    camNear = true; camIdx = bi; camDist = best; camLimit = cams[bi].speedLimit;
    bool over = (curSpeed - camLimit) > SPEED_TOLERANCE;
    if (over && !speeding) {
      snprintf(msg, sizeof(msg), "SPEEDING %d in %d zone", (int)(curSpeed + 0.5f), camLimit);
      addLog(msg, nullptr, 0);
    }
    speeding = over;
  } else {
    if (camNear) addLog("CAMERA_PASSED", nullptr, 0);
    camNear = false; speeding = false; camIdx = -1; camLimit = 0;
  }
}

// ======================= GPS (NMEA) =======================
static bool nmeaChecksumOk(const char *s) {
  if (s[0] != '$') return false;
  uint8_t cs = 0;
  const char *p = s + 1;
  while (*p && *p != '*') cs ^= (uint8_t)*p++;
  if (*p != '*') return false;
  return (uint8_t)strtol(p + 1, NULL, 16) == cs;
}

static int nmeaSplit(char *s, char **f, int maxF) {   // delar på plats vid ',' och '*'
  int n = 0;
  f[n++] = s;
  for (char *p = s; *p; p++) {
    if (*p == '*') { *p = 0; break; }
    if (*p == ',') { *p = 0; if (n >= maxF) break; f[n++] = p + 1; }
  }
  return n;
}

static double nmeaDeg(const char *v) {                // DDMM.MMMM -> grader
  double d = atof(v);
  int deg = (int)(d / 100);
  return deg + (d - deg * 100) / 60.0;
}

static void gpsLost() {
  gpsFix = false;
  curSpeed = 0;
  if (camNear) { camNear = false; speeding = false; camIdx = -1; camLimit = 0; }
}

static void handleNmea(char *s) {
  if (!nmeaChecksumOk(s)) { gpsBadCnt++; return; }
  gpsOkCnt++;
  char *f[20];
  int n = nmeaSplit(s, f, 20);
  if (strlen(f[0]) < 6) return;
  const char *type = f[0] + 3;                        // hoppa över $GP / $GN / $GL ...

  if (!strcmp(type, "GGA") && n >= 8) {
    gpsSats = (uint8_t)atoi(f[7]);
  } else if (!strcmp(type, "RMC") && n >= 8) {
    if (f[2][0] != 'A' || !f[3][0] || !f[5][0]) return;   // A = giltig position
    double la = nmeaDeg(f[3]), lo = nmeaDeg(f[5]);
    if (f[4][0] == 'S') la = -la;
    if (f[6][0] == 'W') lo = -lo;
    float kmh = (float)(atof(f[7]) * 1.852);              // knop -> km/h
    if (kmh < 0 || kmh > 300) kmh = 0;
    curLat = (float)la; curLon = (float)lo; curSpeed = kmh;
    lastFixMs = millis();
    if (!gpsFix) { gpsFix = true; addLog("GPS_FIX", nullptr, 0); }   // inget pip, visas på skärmen
    checkCameras();
  }
}

static void gpsPoll() {
  static char buf[100];
  static uint8_t len = 0;
  uint32_t now = millis();
  while (GPS.available()) {
    char c = (char)GPS.read();
    lastGpsCharMs = now;
    gpsData = true;
    gpsBytes++;
    if (c == '\n' || c == '\r') {
      if (len) {
        buf[len] = 0;
#if GPS_DEBUG >= 2
        Serial.print("GPS< "); Serial.println(buf);
#endif
        handleNmea(buf);
        len = 0;
      }
    } else if (c == '$') {
      buf[0] = c; len = 1;
    } else if (len && len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;                                        // för lång rad eller skräp
    }
  }
  if (gpsData && now - lastGpsCharMs > GPS_TIMEOUT_MS) { gpsData = false; gpsSats = 0; }
  if (gpsFix && now - lastFixMs > GPS_TIMEOUT_MS) { gpsLost(); addLog("GPS_LOST", nullptr, 0); }

#if GPS_DEBUG >= 1
  static uint32_t lastDbg = 0, lastBytes = 0;
  if (now - lastDbg >= 2000) {
    lastDbg = now;
    if (gpsBytes == lastBytes)
      Serial.printf("GPS: INGEN DATA pa GPIO%d (%d baud) - kontrollera GPS TX -> GPIO%d, GND och matning\n",
                    GPS_RX_PIN, GPS_BAUD, GPS_RX_PIN);
    else if (gpsOkCnt == 0)
      Serial.printf("GPS: tar emot tecken (%lu) men inga giltiga NMEA-rader - troligen fel baudrate (nu %d)\n",
                    (unsigned long)gpsBytes, GPS_BAUD);
    else if (gpsFix)
      Serial.printf("GPS: KONTAKT + FIX  sat=%u  lat=%.6f lon=%.6f  %.1f km/h  (rader ok=%lu fel=%lu)\n",
                    (unsigned)gpsSats, curLat, curLon, curSpeed, (unsigned long)gpsOkCnt, (unsigned long)gpsBadCnt);
    else
      Serial.printf("GPS: KONTAKT, vantar pa fix  sat=%u  (rader ok=%lu fel=%lu)\n",
                    (unsigned)gpsSats, (unsigned long)gpsOkCnt, (unsigned long)gpsBadCnt);
    lastBytes = gpsBytes;
  }
#endif
}

// ======================= Sniffer =======================
static void learnFeed(const uint8_t *mac, int8_t rssi) {
  if (mac[0] & 0x01) return;                          // hoppa över broadcast/multicast
  portENTER_CRITICAL(&mux);
  int idx = -1;
  for (uint8_t i = 0; i < learnCount; i++)
    if (memcmp(learnBuf[i].mac, mac, 6) == 0) { idx = i; break; }
  if (idx >= 0) {
    if (rssi > learnBuf[idx].rssi) learnBuf[idx].rssi = rssi;
  } else if (learnCount < LEARN_MAX) {
    memcpy(learnBuf[learnCount].mac, mac, 6);
    learnBuf[learnCount].rssi = rssi;
    learnCount++;
  } else {
    int w = 0;
    for (uint8_t i = 1; i < learnCount; i++)
      if (learnBuf[i].rssi < learnBuf[w].rssi) w = i;
    if (rssi > learnBuf[w].rssi) { memcpy(learnBuf[w].mac, mac, 6); learnBuf[w].rssi = rssi; }
  }
  portEXIT_CRITICAL(&mux);
}

static inline bool matchOui(const uint8_t *mac) {
  for (uint8_t i = 0; i < ouiCount; i++)
    if (mac[0] == ouiTable[i][0] && mac[1] == ouiTable[i][1] && mac[2] == ouiTable[i][2]) return true;
  return false;
}

static void sniffer(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
  const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
  if (pkt->rx_ctrl.sig_len < 24) return;
  const uint8_t *p = pkt->payload;

  if (learning) {                                     // lärläge: samla sändare, ingen RSSI-gräns
    learnFeed(p + 10, pkt->rx_ctrl.rssi);
    return;
  }
  if (pkt->rx_ctrl.rssi < RSSI_MIN) return;

  const uint8_t *hit = nullptr;                       // addr2 = sändare, addr3 = BSSID, addr1 = mottagare
  if (matchOui(p + 10)) hit = p + 10;
  else if (matchOui(p + 16)) hit = p + 16;
  else if (matchOui(p + 4)) hit = p + 4;
  if (hit) {
    for (int i = 0; i < 6; i++) lastMac[i] = hit[i];
    lastRssi = pkt->rx_ctrl.rssi;
    lastSeen = millis();
    if (lastSeen == 0) lastSeen = 1;
  }
}

static void hopTask(void *) {
  uint8_t ch = CH_FIRST;
  for (;;) {
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    curCh = ch;
    ch = (ch >= CH_LAST) ? CH_FIRST : ch + 1;
    vTaskDelay(pdMS_TO_TICKS(DWELL_MS));
  }
}

// ======================= Lärläge =======================
static void startLearn() {
  portENTER_CRITICAL(&mux);
  learnCount = 0;
  portEXIT_CRITICAL(&mux);
  lastSeen = 0;
  learnStart = millis();
  learning = true;
  beepMs(80);
  addLog("LEARN_START", nullptr, 0);
}

static void cancelLearn() {
  learning = false;
  addLog("LEARN_CANCEL", nullptr, 0);
}

static void finishLearn() {
  LearnEntry tmp[LEARN_MAX];
  uint8_t n;
  portENTER_CRITICAL(&mux);
  n = learnCount;
  memcpy(tmp, learnBuf, sizeof(tmp));
  portEXIT_CRITICAL(&mux);

  for (uint8_t i = 0; i + 1 < n; i++)                 // starkast först
    for (uint8_t j = i + 1; j < n; j++)
      if (tmp[j].rssi > tmp[i].rssi) { LearnEntry t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }

  uint8_t added = 0;
  for (uint8_t i = 0; i < n; i++) {
    char m[18], csv[48];
    macStr(tmp[i].mac, m);
    snprintf(csv, sizeof(csv), "%lu,%s,%d\n", (unsigned long)(millis() / 1000), m, tmp[i].rssi);
    sdEnqueue(JOB_LEARN, csv);
    if (addOui(tmp[i].mac)) added++;
  }
  if (added) { saveOuis(); listDirty = true; }

  char msg[40];
  snprintf(msg, sizeof(msg), "LEARN_DONE heard=%u new_oui=%u", n, added);
  addLog(msg, nullptr, 0);

  lastSeen = 0;
  learning = false;
  beepMs(300);
}

// ======================= Skanning + config-AP =======================
static void startScan() {
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_promiscuous(false);
  wifi_promiscuous_filter_t filt;
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&sniffer);
  esp_wifi_set_promiscuous(true);

  static bool hopStarted = false;
  if (!hopStarted) {
    xTaskCreatePinnedToCore(hopTask, "hop", 3072, NULL, 1, NULL, 0);
    hopStarted = true;
  }
  apMode = false;
  lastSeen = 0;
  beepMs(80);
  addLog("SCAN_START", nullptr, 0);
}

static String ouiStr(const uint8_t *o) {
  char b[9];
  snprintf(b, sizeof(b), "%02X:%02X:%02X", o[0], o[1], o[2]);
  return String(b);
}

static int addFromText(const String &text) {
  int before = ouiCount;
  String tok = "";
  String all = text + "\n";
  for (size_t k = 0; k < all.length(); k++) {
    char c = all[k];
    if (c == '\n' || c == '\r' || c == ',' || c == ';' || c == ' ' || c == '\t') {
      if (tok.length()) parseOuiLine(tok);
      tok = "";
    } else {
      tok += c;
    }
  }
  return ouiCount - before;
}

const char PAGE_CSS[] PROGMEM = R"rawliteral(
:root{--bg:#f2f5fa;--card:#fff;--tx:#1b2433;--mut:#66738a;--line:#e1e7f0;--pri:#1f6feb;--dng:#d33f49;--ok:#16794c;--okbg:#e4f6ec}
@media (prefers-color-scheme:dark){:root{--bg:#0e131b;--card:#171e2a;--tx:#e6ebf3;--mut:#93a0b5;--line:#2a3445;--okbg:#12301f;--ok:#5fd394}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--tx);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;line-height:1.45}
.wrap{max-width:560px;margin:0 auto;padding:0 16px}
header{background:#14346b;color:#fff;padding:22px 0 26px}
.hd{display:flex;align-items:center;gap:14px}
.hd h1{margin:0;font-size:24px}.hd p{margin:2px 0 0;font-size:13px;opacity:.85}
main{padding-top:16px}
.stats{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;margin:0 0 14px}
.stat{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:10px 12px;text-align:center}
.stat span{display:block;font-size:11px;text-transform:uppercase;letter-spacing:.6px;color:var(--mut)}
.stat b{font-size:17px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin-bottom:14px}
.card h2{margin:0 0 4px;font-size:17px}.sub{margin:0 0 12px;color:var(--mut);font-size:13px}
.alert{border-radius:10px;padding:10px 14px;margin-bottom:14px;font-size:14px;font-weight:500;background:var(--okbg);color:var(--ok)}
.row{display:flex;align-items:center;justify-content:space-between;padding:9px 0;border-bottom:1px solid var(--line)}
.mac{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:15px}
textarea{width:100%;height:100px;padding:10px;border:1px solid var(--line);border-radius:8px;background:var(--bg);color:var(--tx);font-family:ui-monospace,Menlo,Consolas,monospace;font-size:15px}
.btns{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}
.btn{display:inline-block;padding:9px 16px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--tx);font-size:14px;font-weight:600;text-decoration:none;cursor:pointer}
.pri{background:var(--pri);border-color:var(--pri);color:#fff}.dng{color:var(--dng);border-color:var(--dng)}.sm{padding:4px 10px;font-size:12px}
.note{font-size:12px;color:var(--mut);margin:10px 0 0}
footer{text-align:center;color:var(--mut);font-size:12px;padding:14px 0 26px;border-top:1px solid var(--line);margin-top:10px}
)rawliteral";

static void sendPage(const String &msg = "") {
  String h = "<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'><title>BLUE-WARNER</title><style>";
  h += PAGE_CSS;
  h += "</style></head><body><header><div class='wrap hd'>"
       "<svg viewBox='0 0 24 24' width='38' height='38' fill='none' stroke='#fff' stroke-width='2' stroke-linecap='round'>"
       "<path d='M2 9a15 15 0 0 1 20 0'/><path d='M5 12.5a10.5 10.5 0 0 1 14 0'/><path d='M8.5 16a5.5 5.5 0 0 1 7 0'/><circle cx='12' cy='19.5' r='1' fill='#fff'/></svg>"
       "<div><h1>BLUE-WARNER</h1><p>Passive Wi-Fi OUI detector &middot; Freenove ESP32 Display</p></div></div></header><main class='wrap'>";
  if (msg.length()) h += "<div class='alert'>" + msg + "</div>";
  h += "<div class='stats'><div class='stat'><span>Watched OUIs</span><b>" + String(ouiCount) + " / " + String(MAX_OUI) + "</b></div>"
       "<div class='stat'><span>Storage</span><b>" + String(sdOk ? "Flash + SD" : "Flash only") + "</b></div>"
       "<div class='stat'><span>Speed cameras</span><b>" + String(camCount) + "</b></div>"
       "<div class='stat'><span>Camera list</span><b>" + String(camsFromSd ? "SD card" : (camCount ? "Built-in" : "None")) + "</b></div></div>"
       "<section class='card'><h2>Watched OUIs</h2><p class='sub'>An alarm is raised when a device with one of these MAC prefixes is nearby.</p>";
  for (uint8_t i = 0; i < ouiCount; i++)
    h += "<div class='row'><span class='mac'>" + ouiStr(ouiTable[i]) + "</span><a class='btn dng sm' href='/del?i=" + String(i) + "'>Remove</a></div>";
  if (!ouiCount) h += "<div class='sub'>No OUIs configured.</div>";
  h += "<form method='POST' action='/add'><p class='note'>Add one per line. Full MAC addresses are accepted &ndash; only the first three bytes are stored.</p>"
       "<textarea name='t' placeholder='AA:BB:CC&#10;11:22:33:44:55:66'></textarea>"
       "<div class='btns'><button class='btn pri' type='submit'>Add</button>"
       "<a class='btn' href='/oui_default' onclick=\"return confirm('Restore the default OUI list?')\">Restore default OUIs</a>"
       "<a class='btn dng' href='/clear' onclick=\"return confirm('Remove all OUIs?')\">Clear all</a></div></form></section>"
       "<p class='note' style='text-align:center'>Disconnect from the Wi-Fi when you are done &ndash; scanning starts a couple of seconds later.</p>"
       "</main><footer><div class='wrap'>Developed by <b>A. Isaksson</b> &middot; SA7BNB</div></footer></body></html>";
  server.send(200, "text/html; charset=utf-8", h);
}

static void handleAdd() {
  int n = addFromText(server.arg("t"));
  if (n) { saveOuis(); listDirty = true; addLog("WEB_ADD", nullptr, 0); }
  sendPage("Added " + String(n) + " OUI(s).");
}
static void handleDel() {
  removeOui(server.arg("i").toInt());
  saveOuis();
  listDirty = true;
  addLog("WEB_REMOVE", nullptr, 0);
  sendPage("Removed.");
}
static void handleClear() {
  ouiCount = 0;
  saveOuis();
  listDirty = true;
  addLog("WEB_CLEAR", nullptr, 0);
  sendPage("All OUIs cleared.");
}
static void handleOuiDefault() {
  setDefaults();
  saveOuis();
  listDirty = true;
  addLog("WEB_DEFAULTS", nullptr, 0);
  sendPage("Default OUI list restored.");
}

// ======================= UI-callbacks =======================
static int pendingAction = 0;

static void mboxCb(lv_event_t *e) {
  lv_obj_t *m = lv_event_get_current_target(e);
  uint16_t id = lv_msgbox_get_active_btn(m);
  if (id == 0) {
    if (pendingAction == 1) { setDefaults(); saveOuis(); addLog("DEFAULTS_RESTORED", nullptr, 0); }
    if (pendingAction == 2) { ouiCount = 0; saveOuis(); addLog("LIST_CLEARED", nullptr, 0); }
    listDirty = true;
  }
  lv_msgbox_close(m);
}

static void askConfirm(const char *title, const char *txt, int action) {
  static const char *btns[] = {"Yes", "No", ""};
  pendingAction = action;
  lv_obj_t *m = lv_msgbox_create(NULL, title, txt, btns, false);
  lv_obj_add_event_cb(m, mboxCb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_center(m);
}

static void onLearn(lv_event_t *) {
  if (apMode) { startScan(); return; }                // "Skip" under konfigurations-AP
  if (learning) cancelLearn(); else startLearn();
}
static void setUiState(int st);
static void applyNight() {
  blSet(nightMode ? BL_NIGHT : BL_DAY);
  if (nightOv) {
    if (nightMode) lv_obj_clear_flag(nightOv, LV_OBJ_FLAG_HIDDEN);
    else           lv_obj_add_flag(nightOv, LV_OBJ_FLAG_HIDDEN);
  }
  if (btnNightLbl) lv_label_set_text(btnNightLbl, nightMode ? LV_SYMBOL_EYE_CLOSE " Night ON" : LV_SYMBOL_EYE_OPEN " Night OFF");
  if (uiState != ST_NONE) setUiState(uiState);        // uppdatera RGB-lysdioden
}
static void onNight(lv_event_t *) {
  nightMode = !nightMode;
  prefs.putBool("night", nightMode);
  applyNight();
}
static void onSound(lv_event_t *) {
  soundOn = !soundOn;
  lv_label_set_text(btnSoundLbl, soundOn ? LV_SYMBOL_VOLUME_MAX " Sound ON" : LV_SYMBOL_MUTE " Sound OFF");
}
static void onTest(lv_event_t *) { beepMs(400); }
static void onDefaults(lv_event_t *) { askConfirm("Restore defaults", "Replace the list with the default OUIs?", 1); }
static void onClear(lv_event_t *) { askConfirm("Clear all", "Remove all watched OUIs?", 2); }
static void onDelOui(lv_event_t *e) {
  removeOui((int)(intptr_t)lv_event_get_user_data(e));
  saveOuis();
  listDirty = true;
}

// ======================= UI =======================
static lv_obj_t *mkBtn(lv_obj_t *parent, const char *txt, lv_event_cb_t cb, lv_color_t col, lv_obj_t **lblOut = nullptr) {
  lv_obj_t *b = lv_btn_create(parent);
  lv_obj_set_height(b, 44);
  lv_obj_set_flex_grow(b, 1);
  lv_obj_set_style_bg_color(b, col, 0);
  lv_obj_set_style_radius(b, 10, 0);
  lv_obj_set_style_shadow_width(b, 0, 0);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, FONT_M, 0);
  lv_obj_set_style_text_color(l, lv_color_white(), 0);
  lv_obj_center(l);
  if (lblOut) *lblOut = l;
  return b;
}

static lv_obj_t *mkRow(lv_obj_t *parent) {
  lv_obj_t *r = lv_obj_create(parent);
  lv_obj_set_width(r, LV_PCT(100));
  lv_obj_set_height(r, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(r, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(r, 0, 0);
  lv_obj_set_style_pad_all(r, 0, 0);
  lv_obj_set_style_pad_column(r, 8, 0);
  lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
  lv_obj_clear_flag(r, LV_OBJ_FLAG_SCROLLABLE);
  return r;
}

static void styleTab(lv_obj_t *t) {
  lv_obj_set_style_bg_color(t, COL_BG, 0);
  lv_obj_set_style_bg_opa(t, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all(t, 8, 0);
  lv_obj_set_style_pad_row(t, 8, 0);
  lv_obj_set_flex_flow(t, LV_FLEX_FLOW_COLUMN);
}

static void buildUi() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, COL_BG, 0);
  lv_obj_set_style_pad_all(scr, 0, 0);
  lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // ----- Header -----
  lv_obj_t *hdr = lv_obj_create(scr);
  lv_obj_set_size(hdr, LV_PCT(100), 32);
  lv_obj_set_style_bg_color(hdr, COL_PANEL, 0);
  lv_obj_set_style_radius(hdr, 0, 0);
  lv_obj_set_style_border_width(hdr, 0, 0);
  lv_obj_set_style_pad_hor(hdr, 10, 0);
  lv_obj_set_style_pad_ver(hdr, 0, 0);
  lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *t = lv_label_create(hdr);
  lv_label_set_text(t, LV_SYMBOL_WIFI "  BLUE-WARNER");
  lv_obj_set_style_text_font(t, FONT_L, 0);
  lv_obj_set_style_text_color(t, COL_TEXT, 0);
  lblHdrRight = lv_label_create(hdr);
  lv_label_set_text(lblHdrRight, "");
  lv_obj_set_style_text_font(lblHdrRight, FONT_M, 0);
  lv_obj_set_style_text_color(lblHdrRight, COL_MUTED, 0);

  // ----- Tabs -----
  lv_obj_t *tv = lv_tabview_create(scr, LV_DIR_BOTTOM, 36);
  lv_obj_set_width(tv, LV_PCT(100));
  lv_obj_set_flex_grow(tv, 1);
  lv_obj_set_style_bg_color(tv, COL_BG, 0);
  lv_obj_t *tb = lv_tabview_get_tab_btns(tv);
  lv_obj_set_style_bg_color(tb, COL_PANEL, 0);
  lv_obj_set_style_text_color(tb, COL_MUTED, 0);
  lv_obj_set_style_text_color(tb, COL_TEXT, LV_PART_ITEMS | LV_STATE_CHECKED);
  lv_obj_set_style_border_color(tb, COL_BLUE, LV_PART_ITEMS | LV_STATE_CHECKED);

  lv_obj_t *tabS = lv_tabview_add_tab(tv, LV_SYMBOL_WIFI " Status");
  lv_obj_t *tabO = lv_tabview_add_tab(tv, LV_SYMBOL_LIST " OUIs");
  lv_obj_t *tabL = lv_tabview_add_tab(tv, LV_SYMBOL_FILE " Log");
  styleTab(tabS); styleTab(tabO); styleTab(tabL);
  lv_obj_clear_flag(tabS, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(tabS, LV_FLEX_FLOW_ROW);          // liggande: panel till vänster, knappar till höger
  lv_obj_set_style_pad_column(tabS, 8, 0);

  // ----- Status-flik -----
  panel = lv_obj_create(tabS);
  lv_obj_set_height(panel, LV_PCT(100));
  lv_obj_set_flex_grow(panel, 1);
  lv_obj_set_style_bg_color(panel, COL_PANEL, 0);
  lv_obj_set_style_radius(panel, 18, 0);
  lv_obj_set_style_border_width(panel, 3, 0);
  lv_obj_set_style_border_color(panel, COL_GREEN, 0);
  lv_obj_set_style_pad_all(panel, 10, 0);
  lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

  lblIcon = lv_label_create(panel);
  lv_obj_set_style_text_font(lblIcon, FONT_ICON, 0);
  lblStatus = lv_label_create(panel);
  lv_obj_set_style_text_font(lblStatus, FONT_XL, 0);
  lblNote = lv_label_create(panel);                   // visas vid ALL OK och ALARM
  lv_obj_set_width(lblNote, LV_PCT(100));
  lv_label_set_long_mode(lblNote, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(lblNote, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(lblNote, FONT_L, 0);
  lv_obj_add_flag(lblNote, LV_OBJ_FLAG_HIDDEN);
  lblSub = lv_label_create(panel);
  lv_obj_set_width(lblSub, LV_PCT(100));
  lv_label_set_long_mode(lblSub, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(lblSub, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(lblSub, FONT_M, 0);
  lv_obj_set_style_text_color(lblSub, COL_MUTED, 0);
  barLearn = lv_bar_create(panel);
  lv_obj_set_size(barLearn, LV_PCT(80), 10);
  lv_bar_set_range(barLearn, 0, LEARN_MS / 100);
  lv_obj_set_style_bg_color(barLearn, COL_LINE, 0);
  lv_obj_set_style_bg_color(barLearn, COL_BLUE, LV_PART_INDICATOR);
  lv_obj_add_flag(barLearn, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *col = lv_obj_create(tabS);                    // knappkolumn till höger
  lv_obj_set_size(col, 132, LV_PCT(100));
  lv_obj_set_style_bg_opa(col, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(col, 0, 0);
  lv_obj_set_style_pad_all(col, 0, 0);
  lv_obj_set_style_pad_row(col, 8, 0);
  lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(col, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *b1 = mkBtn(col, LV_SYMBOL_REFRESH " Learn", onLearn, lv_color_hex(0x1F6FEB), &btnLearnLbl);
  lv_obj_t *b2 = mkBtn(col, LV_SYMBOL_VOLUME_MAX " Sound ON", onSound, lv_color_hex(0x2B3A55), &btnSoundLbl);
  lv_obj_t *b3 = mkBtn(col, LV_SYMBOL_PLAY " Test", onTest, lv_color_hex(0x2B3A55));
  lv_obj_t *b4 = mkBtn(col, LV_SYMBOL_EYE_OPEN " Night OFF", onNight, lv_color_hex(0x2B3A55), &btnNightLbl);
  lv_obj_set_width(b1, LV_PCT(100)); lv_obj_set_width(b2, LV_PCT(100));
  lv_obj_set_width(b3, LV_PCT(100)); lv_obj_set_width(b4, LV_PCT(100));

  // Nattläge: svart halvgenomskinligt lager över allt (tar inte emot tryck)
  nightOv = lv_obj_create(lv_layer_top());
  lv_obj_remove_style_all(nightOv);
  lv_obj_set_size(nightOv, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_color(nightOv, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(nightOv, LV_OPA_40, 0);
  lv_obj_clear_flag(nightOv, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_flag(nightOv, LV_OBJ_FLAG_HIDDEN);

  // ----- OUI-flik -----
  lblOuiCount = lv_label_create(tabO);
  lv_obj_set_style_text_font(lblOuiCount, FONT_M, 0);
  lv_obj_set_style_text_color(lblOuiCount, COL_MUTED, 0);
  lv_obj_t *row2 = mkRow(tabO);
  mkBtn(row2, "Restore defaults", onDefaults, lv_color_hex(0x2B3A55));
  mkBtn(row2, "Clear all", onClear, lv_color_hex(0x8A2D33));

  listCont = lv_obj_create(tabO);
  lv_obj_set_width(listCont, LV_PCT(100));
  lv_obj_set_height(listCont, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(listCont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(listCont, 0, 0);
  lv_obj_set_style_pad_all(listCont, 0, 0);
  lv_obj_set_style_pad_row(listCont, 6, 0);
  lv_obj_set_flex_flow(listCont, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(listCont, LV_OBJ_FLAG_SCROLLABLE);

  // ----- Logg-flik -----
  lblLog = lv_label_create(tabL);
  lv_obj_set_width(lblLog, LV_PCT(100));
  lv_label_set_long_mode(lblLog, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_font(lblLog, FONT_M, 0);
  lv_obj_set_style_text_color(lblLog, COL_TEXT, 0);
}

static void rebuildList() {
  lv_obj_clean(listCont);
  for (uint8_t i = 0; i < ouiCount; i++) {
    lv_obj_t *r = lv_obj_create(listCont);
    lv_obj_set_size(r, LV_PCT(100), 42);
    lv_obj_set_style_bg_color(r, COL_PANEL, 0);
    lv_obj_set_style_radius(r, 10, 0);
    lv_obj_set_style_border_width(r, 0, 0);
    lv_obj_set_style_pad_hor(r, 12, 0);
    lv_obj_set_style_pad_ver(r, 0, 0);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(r, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *l = lv_label_create(r);
    lv_label_set_text_fmt(l, "%02X:%02X:%02X", ouiTable[i][0], ouiTable[i][1], ouiTable[i][2]);
    lv_obj_set_style_text_font(l, FONT_L, 0);
    lv_obj_set_style_text_color(l, COL_TEXT, 0);

    lv_obj_t *b = lv_btn_create(r);
    lv_obj_set_size(b, 44, 30);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x8A2D33), 0);
    lv_obj_set_style_radius(b, 8, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_add_event_cb(b, onDelOui, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    lv_obj_t *x = lv_label_create(b);
    lv_label_set_text(x, LV_SYMBOL_CLOSE);
    lv_obj_center(x);
  }
  lv_label_set_text_fmt(lblOuiCount, "%u / %u watched OUIs   -   %s", ouiCount, MAX_OUI,
                        sdOk ? "saved to flash + SD (/oui.txt)" : "saved to flash (no SD card)");
}

static void setUiState(int st) {
  lv_color_t c;
  const char *icon, *txt;
  if (st == ST_AP)         { c = COL_AMBER; icon = LV_SYMBOL_WIFI;    txt = "CONFIG MODE"; rgb(1, 1, 0); }
  else if (st == ST_ALARM) { c = COL_RED;   icon = LV_SYMBOL_WARNING; txt = "ALARM"; rgb(1, 0, 0); }
  else if (st == ST_LEARN) { c = COL_BLUE;  icon = LV_SYMBOL_REFRESH; txt = "LEARN MODE"; rgb(0, 0, 1); }
  else if (st == ST_SPEED) { c = COL_ORANGE; icon = LV_SYMBOL_WARNING; txt = "SLOW DOWN"; rgb(1, 0, 0); }
  else if (st == ST_CAM)   { c = COL_AMBER; icon = LV_SYMBOL_EYE_OPEN; txt = "SPEED CAMERA"; rgb(1, 1, 0); }
  else                     { c = COL_GREEN; icon = LV_SYMBOL_OK;      txt = "ALL OK"; rgb(0, 1, 0); }

  lv_obj_set_style_border_color(panel, c, 0);
  lv_obj_set_style_text_color(lblIcon, c, 0);
  lv_obj_set_style_text_color(lblStatus, c, 0);
  lv_label_set_text(lblIcon, icon);
  lv_label_set_text(lblStatus, txt);
  lv_obj_set_style_text_color(lblNote, c, 0);
  if (st == ST_ALARM || st == ST_OK) {
    lv_label_set_text(lblNote, st == ST_ALARM ? "Blue light nearby" : "No blue light near you");
    lv_obj_clear_flag(lblNote, LV_OBJ_FLAG_HIDDEN);
  } else if (st == ST_SPEED || st == ST_CAM) {
    lv_obj_clear_flag(lblNote, LV_OBJ_FLAG_HIDDEN);     // texten sätts i uiTick
  } else {
    lv_obj_add_flag(lblNote, LV_OBJ_FLAG_HIDDEN);
  }
  lv_obj_set_style_bg_color(barLearn, c, LV_PART_INDICATOR);
  if (st == ST_LEARN || st == ST_AP) lv_obj_clear_flag(barLearn, LV_OBJ_FLAG_HIDDEN);
  else                               lv_obj_add_flag(barLearn, LV_OBJ_FLAG_HIDDEN);
  lv_label_set_text(btnLearnLbl, st == ST_AP ? LV_SYMBOL_NEXT " Skip"
                                 : (st == ST_LEARN ? LV_SYMBOL_CLOSE " Cancel" : LV_SYMBOL_REFRESH " Learn"));
}

// Körs var 100:e ms i LVGL:s tråd
static void uiTick(lv_timer_t *) {
  uint32_t now = millis();
  if (learning && now - learnStart >= LEARN_MS) finishLearn();

  uint32_t seen = lastSeen;
  bool wifiAlarm = !apMode && !learning && seen && now - seen < HOLD_MS;
  int st = apMode ? ST_AP : learning ? ST_LEARN : speeding ? ST_SPEED : wifiAlarm ? ST_ALARM : camNear ? ST_CAM : ST_OK;

  uint8_t mac[6];
  for (int i = 0; i < 6; i++) mac[i] = lastMac[i];
  int rssi = lastRssi;

  static bool prevWifiAlarm = false;                  // loggas oberoende av vad skärmen visar
  if (wifiAlarm && !prevWifiAlarm) addLog("ALARM", mac, rssi);
  else if (!wifiAlarm && prevWifiAlarm) addLog("CLEAR", nullptr, 0);
  prevWifiAlarm = wifiAlarm;

  if (st != uiState) {
    setUiState(st);
    uiState = st;
  }

  int kmh = (int)(curSpeed + 0.5f);

  if (st == ST_SPEED) {
    lv_label_set_text_fmt(lblNote, "Limit %d  -  you %d km/h", camLimit, kmh);
    lv_label_set_text_fmt(lblSub, "Speed camera in %d m\n+%d km/h over the limit%s", (int)camDist, kmh - camLimit,
                          wifiAlarm ? "\n" LV_SYMBOL_WARNING " Blue light nearby" : "");
  } else if (st == ST_CAM) {
    lv_label_set_text_fmt(lblNote, "Limit %d km/h", camLimit);
    lv_label_set_text_fmt(lblSub, "Camera in %d m   -   your speed %d km/h", (int)camDist, kmh);
  } else if (st == ST_ALARM) {
    char m[18];
    macStr(mac, m);
    lv_label_set_text_fmt(lblSub, "%s\nRSSI %d dBm   -   %lu s ago", m, rssi, (unsigned long)((now - seen) / 1000));
  } else if (st == ST_LEARN) {
    uint32_t el = now - learnStart;
    lv_bar_set_value(barLearn, el / 100, LV_ANIM_OFF);
    lv_label_set_text_fmt(lblSub, "Keep the device close\n%lu s left   -   %u MAC heard",
                          (unsigned long)((LEARN_MS - el) / 1000 + 1), (unsigned)learnCount);
  } else if (st == ST_AP) {
    uint32_t el = now - apStart;
    uint32_t rem = el < AP_WAIT_MS ? AP_WAIT_MS - el : 0;
    if (clientSeen) {
      lv_obj_add_flag(barLearn, LV_OBJ_FLAG_HIDDEN);
      lv_label_set_text(lblSub, WiFi.softAPgetStationNum() > 0
        ? "Client connected\nOpen http://192.168.4.1\nDisconnect from the Wi-Fi when done"
        : "Client disconnected\nScanning starts now");
    } else {
      lv_obj_clear_flag(barLearn, LV_OBJ_FLAG_HIDDEN);
      lv_bar_set_value(barLearn, el / 100, LV_ANIM_OFF);
      lv_label_set_text_fmt(lblSub, "Wi-Fi: %s   Password: %s\nOpen http://192.168.4.1\nScanning starts in %lu s",
                            AP_SSID, AP_PASS, (unsigned long)(rem / 1000 + 1));
    }
  } else {
    if (gpsFix)
      lv_label_set_text_fmt(lblSub, "Scanning channel %u   -   %u OUIs watched\n%d km/h   -   %d cameras loaded",
                            (unsigned)curCh, ouiCount, kmh, camCount);
    else
      lv_label_set_text_fmt(lblSub, "Scanning channel %u   -   %u OUIs watched\n%s   -   %d cameras loaded",
                            (unsigned)curCh, ouiCount, gpsData ? "Waiting for GPS fix" : "No GPS data", camCount);
  }

  // Ljud: Wi-Fi-larm pulserar på BEEP_FREQ, fartvarning ger korta pip på SPEED_FREQ
  // (tätare ju större överträdelse), kort pip för händelser.
  if (speeding) {
    float over = curSpeed - camLimit;
    uint32_t interval = over <= 5 ? 2000 : over <= 10 ? 1000 : over <= 20 ? 500 : 300;
    if (now - lastSpeedBeep >= interval) { lastSpeedBeep = now; speedBeepUntil = now + 120; }
  }
  uint16_t tone = 0;
  if (soundOn) {
    if (wifiAlarm && (now / 250) % 2 == 0) tone = BEEP_FREQ;
    if (now < beepUntil) tone = BEEP_FREQ;
    if (speeding && now < speedBeepUntil) tone = SPEED_FREQ;
  }
  spk(tone);

  if (listDirty) { listDirty = false; rebuildList(); }
  if (logDirty)  { logDirty = false;  lv_label_set_text(lblLog, logText.length() ? logText.c_str() : "No events yet."); }

  static uint32_t lastHdr = 0;
  if (now - lastHdr > 1000) {
    lastHdr = now;
    char g[24];
    if (gpsFix)       snprintf(g, sizeof(g), "FIX %u sat  %d km/h", (unsigned)gpsSats, kmh);
    else if (gpsData) snprintf(g, sizeof(g), "no fix (%u sat)", (unsigned)gpsSats);
    else              snprintf(g, sizeof(g), "no GPS");
    lv_label_set_text_fmt(lblHdrRight, "%s %s   %s %s", LV_SYMBOL_GPS, g, LV_SYMBOL_SD_CARD, sdOk ? "SD OK" : "no SD");
    lv_obj_set_style_text_color(lblHdrRight, gpsFix ? COL_GREEN : COL_MUTED, 0);   // grön = GPS-fix
  }
}

// ======================= Arduino =======================
void setup() {
  Serial.begin(115200);

  pinMode(RGB_R, OUTPUT); pinMode(RGB_G, OUTPUT); pinMode(RGB_B, OUTPUT);
  rgb(0, 1, 0);
  pinMode(AUDIO_EN, OUTPUT);
  digitalWrite(AUDIO_EN, LOW);                        // slå på högtalarförstärkaren
  spkInit();

  screen.init();                                      // Freenove: TFT + touch + LVGL

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOk = SD.begin(SD_CS, SPI, 20000000);
  Serial.println(sdOk ? "SD card OK" : "SD card not found");

  prefs.begin("ouiw", false);                         // flash-kopia av listan
  if (sdOk) {
    sdQ = xQueueCreate(24, sizeof(SdJob));
    xTaskCreatePinnedToCore(sdTask, "sd", 4096, NULL, 1, NULL, 0);
  }
  loadOuis();                                         // SD -> RAM (+ spegling till flash)
  loadCameras();                                      // SD -> RAM, annars inbyggd lista

  GPS.setRxBufferSize(1024);                          // tål att LVGL ritar en stund
  GPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);    // endast RX

  blInit();
  nightMode = prefs.getBool("night", false);          // nattläge sparas i flash
  buildUi();
  applyNight();
  addLog("BOOT", nullptr, 0);
  {
    char msg[40];
    snprintf(msg, sizeof(msg), "CAMERAS n=%d src=%s", camCount, camsFromSd ? "SD" : (camCount ? "builtin" : "none"));
    addLog(msg, nullptr, 0);
  }

  WiFi.mode(WIFI_AP);                                 // först: konfigurations-AP, skanning startar efter
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", HTTP_GET, []() { sendPage(); });
  server.on("/add", HTTP_POST, handleAdd);
  server.on("/del", HTTP_GET, handleDel);
  server.on("/clear", HTTP_GET, handleClear);
  server.on("/oui_default", HTTP_GET, handleOuiDefault);
  server.onNotFound([]() { sendPage(); });
  server.begin();
  apMode = true;
  apStart = millis();
  addLog("AP_START", nullptr, 0);

  lv_timer_create(uiTick, 100, NULL);
}

void loop() {
  gpsPoll();                                          // GPS + kamerakontroll (även under config-AP)
  if (apMode) {
    server.handleClient();
    uint32_t now = millis();
    if (WiFi.softAPgetStationNum() > 0) { clientSeen = true; lastClientMs = now; }
    if (!clientSeen && now - apStart >= AP_WAIT_MS) startScan();
    else if (clientSeen && now - lastClientMs >= AP_DROP_MS) startScan();
  }
  screen.routine();                                   // LVGL + touch
  delay(5);
}
