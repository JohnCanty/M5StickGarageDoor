/**
 * @file    main.cpp
 * @brief   M5CycleHomeZero – GPS-triggered automatic garage door controller
 * @author  John Canty (https://github.com/JohnCanty)
 *
 * Runs on an M5StickC Plus (ESP32) paired with an AT6668 GPS/BDS module on a
 * SoftwareSerial port and two relay outputs.
 *
 * Boot-mode state machine (persisted in NVS "counter" key):
 *
 *   Value 0       – Mode 0: GPS-only loop.
 *                   Relay1 is driven HIGH while speed < speedSP (i.e. at low
 *                   speed / stopped).  Approach detection counts consecutive
 *                   readings that are closer than the previous one; once
 *                   approachCountSP counts are logged inside the distanceSP
 *                   zone the device reboots into Mode 2.
 *
 *   Value 1..99   – Mode 1: Close-door sequence.
 *                   Connects to WiFi/MQTT, waits for a BtnA press or an IMU
 *                   movement event, then publishes a "clos" command and waits
 *                   for the garage to confirm closed before rebooting into
 *                   Mode 0.
 *
 *   Value >= 100  – Mode 2: Open-door sequence.
 *                   Connects to WiFi/MQTT (retries forever), publishes an
 *                   "open" command, waits for confirmation, then powers off
 *                   so the device wakes again in Mode 1 on the next button
 *                   press or barrel-connector insertion.
 *
 * Initial configuration is served via a captive Wi-Fi AP (SSID "M5CycleSetup")
 * using an embedded dark-theme HTML form.  All credentials are stored in NVS
 * with XOR encryption keyed on the device's unique EFuse MAC address.
 *
 * Hardware:
 *   Board  : M5StickC Plus (ESP32 240 MHz, 4 MB flash)
 *   GPS    : AT6668 on SoftwareSerial RX=33 TX=32
 *   Relay1 : GPIO 26  – speed-gated relay (HIGH = activated)
 *   Relay2 : GPIO 25  – reserved for future use
 */
#include <Arduino.h>
#include <cmath>
#include <cstring>
#include <M5StickCPlus.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <WebServer.h>

// ─── Hardware ─────────────────────────────────────────────────────────────────
// Physical pin assignments and low-level timing constants.
constexpr int           RXPin              = 33, TXPin = 32;  // GPS SoftwareSerial (primary mapping)
constexpr int           Relay1             = 26, Relay2 = 25; // Output relay GPIOs
constexpr uint32_t      GPSBaud            = 9600;            // Fallback / default GPS baud
constexpr unsigned long GPSDetectTimeoutMs = 1200;            // Per-setting scan window (ms)

// ─── Tunables ─────────────────────────────────────────────────────────────────
// Adjust these without touching application logic.
constexpr float  speedSP         = 10.0f; // mph  – Relay1 activates when speed is below this
constexpr double distanceSP      = 500.0; // m    – radius around home that triggers approach logic
constexpr int    approachCountSP = 3;     // consecutive "getting closer" GPS readings before triggering door
constexpr unsigned long approachWindowMs = 5UL * 60UL * 1000UL; // 5-minute holdoff before mode-switch decisions

// ─── Time ─────────────────────────────────────────────────────────────────────
// Pacific Time: UTC-8 standard, +1 h DST.
constexpr long gmtOffset_sec      = -28800; // UTC-8 (PST)
constexpr int  daylightOffset_sec = 3600;   // +1 h during PDT

// ─── Runtime config (loaded from encrypted NVS) ───────────────────────────────
// All fields are populated by loadConfig() on every boot from the "config" NVS
// namespace.  Empty strings indicate that the config portal has not yet been
// completed and loadConfig() will return false, causing setup() to launch the
// portal automatically.
static char wifiSsid[64]      = {};
static char wifiPass[64]      = {};
static char mqttServer[64]    = {};
static char ntpServer[64]     = {};
static char mqttBaseTopic[64] = {};
static char mqttGaragePub[64] = {};
static char mqttGarageSub[64] = {};
static char homeLat[24]       = {};
static char homeLon[24]       = {};
static char gpsBaudStr[16]    = {};  // Persisted GPS baud rate (e.g. "38400")
static char gpsConfigStr[8]   = {};  // Persisted GPS frame config (e.g. "8N1")
static char gpsRxPinStr[8]    = {};  // Persisted GPS RX pin (e.g. "33")
static char gpsTxPinStr[8]    = {};  // Persisted GPS TX pin (e.g. "32")
static char topicLWT[128]     = {};
static char topicArrived[128] = {};

// ─── State ────────────────────────────────────────────────────────────────────
// All mutable runtime state.  Initialised to safe defaults; overwritten from
// NVS or GPS on each iteration.
static int    value           = 0;     // Current boot mode (see file header)
static bool   away            = false; // Persisted away flag (NVS "away")
static bool   doorOpen        = false; // Set true by MQTT callback on "open" message
static bool   doorClos        = false; // Set true by MQTT callback on "clos" message
static float  currentSpeed    = 0.0f;  // Latest GPS speed in mph
static double currentDistance = 0.0;  // Latest distance from home in metres
static double lastDistance    = 0.0;  // Previous distance sample for approach detection
static int    distanceCount   = 0;    // Consecutive closer-to-home GPS readings
static bool   mode0ApproachGateActive = false; // Enabled when entering Mode 0 from Mode 1 (away=true)
static bool   mode0WindowInitialized  = false; // True once first valid distance is captured
static unsigned long mode0WindowStartMs = 0;   // millis() at start of current 5-minute window
static double mode0WindowStartDistance = 0.0;  // Distance at start of current 5-minute window
static bool   mode0SkipWaitOnBoot    = false;  // Persisted bypass for holdoff after mode0 USB-loss resume
static bool   mode0UsbWasPresent     = false;  // Previous USB power state for edge detection

// Detected GPS serial parameters – updated by initGpsSerial() and used by
// getGPSData() for all subsequent reads.
static uint32_t                  detectedGpsBaud   = GPSBaud;
static EspSoftwareSerial::Config detectedGpsConfig = SWSERIAL_8N1;
static int                       activeGpsRxPin    = RXPin;
static int                       activeGpsTxPin    = TXPin;

// ─── Objects ──────────────────────────────────────────────────────────────────
// Library object instances.  Declared at file scope so they are accessible
// from all functions without passing pointers.
WiFiClient     espClient;
PubSubClient   client(espClient);
Preferences    preferences;
TinyGPSPlus    gps;
SoftwareSerial ss(RXPin, TXPin);

// ─── Forward declarations ─────────────────────────────────────────────────────
// Required because the Arduino/PlatformIO C++ build does not auto-generate
// prototypes as the .ino toolchain does.
bool   hasGpsFix();
void   drawGpsFixIcon();
const char* gpsConfigName(EspSoftwareSerial::Config config);
bool   looksLikeNmeaSentence(const char* s);
bool   tryGpsSerialSetting(uint32_t baud, EspSoftwareSerial::Config cfg, unsigned long ms,
                           int rxPin, int txPin);
bool   initGpsSerial();
void   persistDetectedGpsSettings();
void   setupWifi(bool infiniteRetry = false);
void   ntpGetLocalTime();
void   callback(char* topic, byte* payload, unsigned int length);
void   reConnect();
double distanceToHome(double lat, double lon);
void   toggleDoor(const char* arrivedMsg, const char* cmdMsg, bool& doorFlag);
void   openDoor();
void   closeDoor();
void   restartCleanup(bool powerOff);
void   resetCounter(int x);
void   awayCheck(bool x);
void   handleDistanceChange();
void   getGPSData(unsigned long ms);
bool   isUsbPowerPresent();
bool   loadConfig();
void   saveConfig(const char* ssid, const char* pass, const char* mqtt,
                  const char* ntp,  const char* base,
                  const char* garagePub, const char* garageSub,
                  const char* lat,  const char* lon,
                  const char* gpsBaud, const char* gpsConfig);
void   runConfigPortal();

// ─── Encrypted config storage (XOR with device EFuse MAC) ────────────────────
// Credentials are stored in NVS as XOR-ciphered byte blobs keyed on the 6-byte
// EFuse MAC address.  This prevents casual extraction of WiFi/MQTT passwords
// via a simple NVS dump while keeping the implementation compact.
//
// Security note: XOR with a static device-unique key provides obfuscation, not
// cryptographic security.  An attacker with physical access to the chip could
// recover the key via ESP32 eFuse read commands.  For a higher-security
// deployment, replace with AES-128 using a key derived from the MAC + a secret.

/**
 * @brief XOR-cipher helper.  Applies the repeating-key XOR transform in-place.
 *
 * Encryption and decryption are the same operation (XOR is its own inverse).
 *
 * @param key     Pointer to the key bytes.
 * @param keyLen  Length of the key in bytes (6 for EFuse MAC).
 * @param in      Input data.
 * @param out     Output buffer (may alias in for in-place operation).
 * @param len     Number of bytes to process.
 */
static void xorCrypt(const uint8_t* key, size_t keyLen,
                     const uint8_t* in, uint8_t* out, size_t len)
{
  for (size_t i = 0; i < len; i++)
    out[i] = in[i] ^ key[i % keyLen];
}

/**
 * @brief Encrypts @p value and writes it to the currently-open Preferences
 *        namespace under @p nvKey.
 *
 * Truncates to 64 bytes if the string is longer (no config field should be).
 *
 * @param nvKey  NVS key string.
 * @param value  Plaintext string to store.
 */
static void encryptStore(const char* nvKey, const char* value)
{
  uint64_t mac    = ESP.getEfuseMac();
  size_t   len    = strlen(value);
  uint8_t  buf[64] = {};
  if (len > sizeof(buf)) len = sizeof(buf);
  xorCrypt((const uint8_t*)&mac, 6, (const uint8_t*)value, buf, len);
  preferences.putBytes(nvKey, buf, len);
}

/**
 * @brief Reads and decrypts a byte blob from the currently-open Preferences
 *        namespace.
 *
 * @param nvKey   NVS key string.
 * @param out     Destination character buffer.
 * @param outMax  Size of @p out in bytes (including the null terminator).
 * @return true   Value was present and decrypted successfully.
 * @return false  Key was not found; @p out is set to an empty string.
 */
static bool decryptLoad(const char* nvKey, char* out, size_t outMax)
{
  uint64_t mac   = ESP.getEfuseMac();
  uint8_t  buf[64] = {};
  size_t   len   = preferences.getBytes(nvKey, buf, sizeof(buf));
  if (len == 0) { out[0] = '\0'; return false; }
  if (len >= outMax) len = outMax - 1;
  xorCrypt((const uint8_t*)&mac, 6, buf, (uint8_t*)out, len);
  out[len] = '\0';
  return true;
}

/**
 * @brief Loads all configuration fields from encrypted NVS and assembles the
 *        derived MQTT topic strings. Also reads persisted GPS configuration.
 *
 * Opens the "config" namespace in read-only mode, decrypts each field, then
 * builds topicLWT and topicArrived by appending the suffixes to mqttBaseTopic.
 * GPS configuration fields (baud rate, frame format) are also loaded. The WiFi
 * password is optional so open networks remain a valid configuration.
 *
 * @return true   All required fields were present and decrypted.
 * @return false  One or more fields were missing; the device should enter the
 *                config portal.
 */
bool loadConfig()
{
  if (!preferences.begin("config", true)) {
    wifiSsid[0] = '\0';
    wifiPass[0] = '\0';
    mqttServer[0] = '\0';
    ntpServer[0] = '\0';
    mqttBaseTopic[0] = '\0';
    mqttGaragePub[0] = '\0';
    mqttGarageSub[0] = '\0';
    homeLat[0] = '\0';
    homeLon[0] = '\0';
    gpsBaudStr[0] = '\0';
    gpsConfigStr[0] = '\0';
    gpsRxPinStr[0] = '\0';
    gpsTxPinStr[0] = '\0';
    return false;
  }
  decryptLoad("wpass", wifiPass, sizeof(wifiPass));
  bool ok =
    decryptLoad("wssid", wifiSsid,      sizeof(wifiSsid))      &&
    decryptLoad("mqtt",  mqttServer,    sizeof(mqttServer))    &&
    decryptLoad("ntp",   ntpServer,     sizeof(ntpServer))     &&
    decryptLoad("base",  mqttBaseTopic, sizeof(mqttBaseTopic)) &&
    decryptLoad("gpub",  mqttGaragePub, sizeof(mqttGaragePub)) &&
    decryptLoad("gsub",  mqttGarageSub, sizeof(mqttGarageSub)) &&
    decryptLoad("hlat",  homeLat,       sizeof(homeLat))       &&
    decryptLoad("hlon",  homeLon,       sizeof(homeLon));
  // GPS configuration is stored unencrypted (non-sensitive)
  preferences.getString("gpsbaud", gpsBaudStr, sizeof(gpsBaudStr));
  preferences.getString("gpsconfig", gpsConfigStr, sizeof(gpsConfigStr));
  preferences.getString("gpsrx", gpsRxPinStr, sizeof(gpsRxPinStr));
  preferences.getString("gpstx", gpsTxPinStr, sizeof(gpsTxPinStr));
  preferences.end();
  if (ok) {
    snprintf(topicLWT,     sizeof(topicLWT),     "%sLWT",     mqttBaseTopic);
    snprintf(topicArrived, sizeof(topicArrived), "%sArrived", mqttBaseTopic);
  }
  return ok;
}

/**
 * @brief Encrypts and persists all configuration fields to NVS, including
 *        detected GPS configuration.
 *
 * Called from handleConfigSave() after the web form is submitted.  The
 * "config" namespace is opened in read-write mode, each field encrypted with
 * the device MAC key, and the namespace closed (flushed) before returning.
 * GPS baud rate and frame format are stored unencrypted.
 *
 * @param ssid       WiFi network SSID.
 * @param pass       WiFi network password.
 * @param mqtt       MQTT broker hostname or IP address.
 * @param ntp        NTP server hostname or IP address.
 * @param base       MQTT base topic prefix (e.g. "Motorcycle/Zero/").
 * @param garagePub  MQTT topic to publish garage door commands to.
 * @param garageSub  MQTT topic to subscribe to for garage door status.
 * @param lat        Home latitude in decimal degrees (e.g. "37.2431").
 * @param lon        Home longitude in decimal degrees (e.g. "-115.7930").
 * @param gpsBaud    Detected GPS baud rate as string (e.g. "38400").
 * @param gpsConfig  Detected GPS frame format (e.g. "8N1").
 */
void saveConfig(const char* ssid, const char* pass, const char* mqtt,
                const char* ntp,  const char* base,
                const char* garagePub, const char* garageSub,
                const char* lat,  const char* lon,
                const char* gpsBaud, const char* gpsConfig)
{
  if (!preferences.begin("config", false)) {
    return;
  }
  encryptStore("wssid", ssid);
  encryptStore("wpass", pass);
  encryptStore("mqtt",  mqtt);
  encryptStore("ntp",   ntp);
  encryptStore("base",  base);
  encryptStore("gpub",  garagePub);
  encryptStore("gsub",  garageSub);
  encryptStore("hlat",  lat);
  encryptStore("hlon",  lon);
  preferences.putString("gpsbaud", gpsBaud);
  preferences.putString("gpsconfig", gpsConfig);
  preferences.putString("gpsrx", String(activeGpsRxPin).c_str());
  preferences.putString("gpstx", String(activeGpsTxPin).c_str());
  preferences.end();
}

/**
 * @brief Persists the last known-good GPS serial settings for future boots.
 *
 * Uses a dedicated Preferences instance so GPS settings can be updated even
 * while the global Preferences object is holding the "lastAction" namespace.
 */
void persistDetectedGpsSettings()
{
  char gpsBaudLocal[16] = {};
  char gpsConfigLocal[8] = {};
  char gpsRxLocal[8] = {};
  char gpsTxLocal[8] = {};
  snprintf(gpsBaudLocal, sizeof(gpsBaudLocal), "%lu", detectedGpsBaud);
  snprintf(gpsConfigLocal, sizeof(gpsConfigLocal), "%s", gpsConfigName(detectedGpsConfig));
  snprintf(gpsRxLocal, sizeof(gpsRxLocal), "%d", activeGpsRxPin);
  snprintf(gpsTxLocal, sizeof(gpsTxLocal), "%d", activeGpsTxPin);

  Preferences gpsPrefs;
  if (!gpsPrefs.begin("config", false)) {
    return;
  }
  gpsPrefs.putString("gpsbaud", gpsBaudLocal);
  gpsPrefs.putString("gpsconfig", gpsConfigLocal);
  gpsPrefs.putString("gpsrx", gpsRxLocal);
  gpsPrefs.putString("gpstx", gpsTxLocal);
  gpsPrefs.end();

  snprintf(gpsBaudStr, sizeof(gpsBaudStr), "%s", gpsBaudLocal);
  snprintf(gpsConfigStr, sizeof(gpsConfigStr), "%s", gpsConfigLocal);
  snprintf(gpsRxPinStr, sizeof(gpsRxPinStr), "%s", gpsRxLocal);
  snprintf(gpsTxPinStr, sizeof(gpsTxPinStr), "%s", gpsTxLocal);
}

// ─── Config portal ────────────────────────────────────────────────────────────
// On first boot (or when BtnA is held at power-on) the device initializes the
// GPS module, then starts a Wi-Fi soft-AP named "M5CycleSetup" and serves a
// configuration form at http://192.168.4.1. The form displays live GPS position,
// satellite count, and detected serial configuration. Users can click "Set Here"
// buttons next to latitude/longitude fields to populate coordinates from current
// GPS position, or manually enter values. The form POSTs to /save which persists
// all fields (including detected GPS configuration) to NVS and reboots into
// normal operation. On future boots, the stored GPS configuration is tried first
// before performing a full serial scan.
//
// The CONFIG_HTML and SAVED_HTML strings are stored in flash (const char[]) to
// minimise heap usage; GPS status metadata is injected dynamically at runtime.
static const char CONFIG_AP_SSID[] = "M5CycleSetup";
static WebServer  _cfgServer(80);
static bool       _cfgSaved = false;
// GPS status captured during portal operation for web display
static char _portalGpsPos[32]   = "N/A";
static char _portalGpsSats[16]  = "N/A";
static char _portalGpsConfig[16] = "N/A";

static const char CONFIG_HTML[] =
  "<!DOCTYPE html><html><head>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>M5Cycle Setup</title>"
  "<style>"
  "body{font-family:sans-serif;max-width:500px;margin:auto;padding:16px;"
       "background:#1a1a1a;color:#eee}"
  "h2{color:#4CAF50;margin-bottom:4px}"
  "p.sub{color:#888;font-size:13px;margin-top:0}"
  "label{display:block;margin-top:14px;font-size:13px;color:#aaa}"
  "input{width:100%;padding:8px 10px;margin-top:4px;background:#2a2a2a;"
        "border:1px solid #555;color:#fff;border-radius:4px;box-sizing:border-box}"
  ".hint{font-size:11px;color:#666;margin-top:3px}"
  "button{width:100%;padding:13px;margin-top:22px;background:#4CAF50;"
          "color:#fff;border:none;border-radius:4px;font-size:16px;cursor:pointer}"
  "button:hover{background:#45a049}"
  "</style></head><body>"
  "<h2>M5Cycle Setup</h2>"
  "<p class='sub'>Connect to <b>M5CycleSetup</b> then open 192.168.4.1</p>"
  "<form action='/save' method='POST'>"
  "<label>WiFi SSID</label>"
  "<input name='ssid' maxlength='63' required>"
  "<label>WiFi Password</label>"
  "<input name='pass' type='password' maxlength='63'>"
  "<label>MQTT Server</label>"
  "<input name='mqtt' placeholder='10.10.11.55' maxlength='63' required>"
  "<label>NTP Server</label>"
  "<input name='ntp' placeholder='pool.ntp.org' maxlength='63' required>"
  "<label>Motorcycle Base MQTT Topic</label>"
  "<input name='base' placeholder='Motorcycle/Zero/' maxlength='63' required>"
  "<p class='hint'>LWT and Arrived will be appended &mdash; e.g. Motorcycle/Zero/LWT</p>"
  "<label>Garage Door Command Topic (publish)</label>"
  "<input name='gpub' placeholder='GarageDoor/CMND' maxlength='63' required>"
  "<label>Garage Door Status Topic (subscribe)</label>"
  "<input name='gsub' placeholder='GarageDoor/Status' maxlength='63' required>"
  "<label>Home Latitude</label>"
  "<input name='hlat' placeholder='32.826940' maxlength='20' required>"
  "<label>Home Longitude</label>"
  "<input name='hlon' placeholder='-117.227327' maxlength='20' required>"
  "<p class='hint'>Decimal degrees, e.g. 32.826940 / -117.227327</p>"
  "<button type='submit'>Save &amp; Restart</button>"
  "</form></body></html>";

static const char SAVED_HTML[] =
  "<!DOCTYPE html><html><head>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<style>body{font-family:sans-serif;background:#1a1a1a;color:#eee;"
              "text-align:center;padding:40px}</style>"
  "</head><body>"
  "<h2 style='color:#4CAF50'>&#10003; Saved!</h2>"
  "<p>Device is restarting&hellip;</p>"
  "</body></html>";

/** @brief Serves the initial configuration form (GET /), including current GPS status.  */
static void handleConfigRoot()
{
  String html = CONFIG_HTML;
  // Insert GPS status section with "Set Here" buttons before the form
  String gpsSection = 
    "<h3 style='color:#4CAF50;margin-top:20px'>GPS Status</h3>"
    "<p style='background:#2a2a2a;padding:10px;border-radius:4px;font-size:12px'>"
    "Position: <b id='gpspos'>" + String(_portalGpsPos) + "</b><br/>"
    "Satellites: <b id='gpssats'>" + String(_portalGpsSats) + "</b><br/>"
    "Config: <b>" + String(_portalGpsConfig) + "</b>"
    "</p>"
    "<script>"
    "function setHomeLat(){var lat=document.getElementById('gpspos').textContent.split(',')[0];document.getElementById('hlat').value=lat.trim();}"
    "function setHomeLon(){var lon=document.getElementById('gpspos').textContent.split(',')[1];document.getElementById('hlon').value=lon.trim();}"
    "</script>";
  
  // Replace placeholder in form - insert before Home Latitude
  int insertPos = html.indexOf("Home Latitude");
  if (insertPos > 0) {
    html = html.substring(0, insertPos) + gpsSection + html.substring(insertPos);
  }
  
  // Modify the home coordinate field section to add buttons
  String latButtonHtml = 
    "<label>Home Latitude</label>"
    "<div style='display:flex;gap:5px'>"
    "<input id='hlat' name='hlat' placeholder='37.2431' maxlength='20' required style='flex:1'>"
    "<button type='button' onclick='setHomeLat()' style='width:auto;padding:8px 12px;margin-top:4px;font-size:12px'>Set Here</button>"
    "</div>";
  
  String lonButtonHtml = 
    "<label>Home Longitude</label>"
    "<div style='display:flex;gap:5px'>"
    "<input id='hlon' name='hlon' placeholder='-115.7930' maxlength='20' required style='flex:1'>"
    "<button type='button' onclick='setHomeLon()' style='width:auto;padding:8px 12px;margin-top:4px;font-size:12px'>Set Here</button>"
    "</div>";
  
  // Replace the latitude field
  int latPos = html.indexOf("<label>Home Latitude</label>");
  if (latPos > 0) {
    int latEnd = html.indexOf("</label>", latPos) + 8;
    latEnd = html.indexOf("<input", latEnd);
    if (latEnd > 0) {
      int latInputEnd = html.indexOf(">", latEnd) + 1;
      html = html.substring(0, latPos) + latButtonHtml + html.substring(latInputEnd);
    }
  }
  
  // Replace the longitude field with similar approach
  lonButtonHtml = 
    "<p class='hint'>Decimal degrees, e.g. 37.2431 / -115.7930</p>"
    "<label>Home Longitude</label>"
    "<div style='display:flex;gap:5px'>"
    "<input id='hlon' name='hlon' placeholder='-115.7930' maxlength='20' required style='flex:1'>"
    "<button type='button' onclick='setHomeLon()' style='width:auto;padding:8px 12px;margin-top:4px;font-size:12px'>Set Here</button>"
    "</div>";
  
  int lonPos = html.indexOf("<label>Home Longitude</label>");
  if (lonPos > 0) {
    int lonEnd = html.indexOf("</label>", lonPos) + 8;
    lonEnd = html.indexOf("<input", lonEnd);
    if (lonEnd > 0) {
      // Find the end of the hint text that follows
      int hintEnd = html.indexOf("</p>", lonEnd);
      if (hintEnd > 0) {
        html = html.substring(0, lonPos) + lonButtonHtml + html.substring(hintEnd);
      }
    }
  }
  
  _cfgServer.send(200, "text/html", html.c_str());
}

/**
 * @brief Handles the POST /save request from the configuration form.
 *
 * Validates that all required fields are present and non-empty, captures the
 * current GPS configuration, renders the "Saved!" confirmation page, persists
 * the submitted values, and sets the _cfgSaved flag so runConfigPortal() exits
 * its event loop and reboots.
 */
static void handleConfigSave()
{
  static const char* required[] = {"ssid","mqtt","ntp","base","gpub","gsub","hlat","hlon"};
  for (const char* f : required) {
    if (!_cfgServer.hasArg(f) || _cfgServer.arg(f).isEmpty()) {
      _cfgServer.send(400, "text/plain", "Missing required fields");
      return;
    }
  }
  
  // Capture GPS configuration at save time
  char gpsBaudStr_local[16] = {};
  char gpsConfigStr_local[8] = {};
  snprintf(gpsBaudStr_local, sizeof(gpsBaudStr_local), "%lu", detectedGpsBaud);
  snprintf(gpsConfigStr_local, sizeof(gpsConfigStr_local), "%s", gpsConfigName(detectedGpsConfig));
  
  _cfgServer.send(200, "text/html", SAVED_HTML);
  saveConfig(
    _cfgServer.arg("ssid").c_str(), _cfgServer.arg("pass").c_str(),
    _cfgServer.arg("mqtt").c_str(), _cfgServer.arg("ntp").c_str(),
    _cfgServer.arg("base").c_str(), _cfgServer.arg("gpub").c_str(),
    _cfgServer.arg("gsub").c_str(), _cfgServer.arg("hlat").c_str(),
    _cfgServer.arg("hlon").c_str(),
    gpsBaudStr_local, gpsConfigStr_local
  );
  _cfgSaved = true;
}

/**
 * @brief Starts the configuration AP and blocks until the form is saved or
 *        BtnA is pressed, then reboots the device.
 *
 * Initializes the GPS module during startup so the user can see live position
 * and satellite data on the configuration form. This function never returns.\n * It is called from setup() when the device has no saved configuration or the\n * user holds BtnA at boot.  After a successful save (or a BtnA skip),\n * restartCleanup(false) triggers a soft reset so the device re-enters setup()\n * and loads the new config.\n */
void runConfigPortal()
{
  // Initialize GPS before starting web server
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.println("Initializing GPS...");
  initGpsSerial();
  snprintf(_portalGpsConfig, sizeof(_portalGpsConfig), "%s @ %lu",
           gpsConfigName(detectedGpsConfig), detectedGpsBaud);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(CONFIG_AP_SSID);
  IPAddress ip = WiFi.softAPIP();

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.println("== Config Mode ==");
  M5.Lcd.println("Connect WiFi to:");
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.println(CONFIG_AP_SSID);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.println("Then open:");
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.println(ip.toString());
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.println("\nBtnA: skip setup");
  M5.Lcd.println("and reboot");

  _cfgServer.on("/",     HTTP_GET,  handleConfigRoot);
  _cfgServer.on("/save", HTTP_POST, handleConfigSave);
  _cfgServer.onNotFound([]() {
    _cfgServer.sendHeader("Location", "/", true);
    _cfgServer.send(302, "text/plain", "");
  });
  _cfgServer.begin();

  static unsigned long lastGpsUpdate = 0;
  while (!_cfgSaved)
  {
    M5.update();
    _cfgServer.handleClient();
    if (M5.BtnA.wasPressed()) { break; }
    
    // Update GPS status every 500ms
    if (millis() - lastGpsUpdate > 500) {
      lastGpsUpdate = millis();
      getGPSData(100);
      
      if (gps.location.isValid()) {
        snprintf(_portalGpsPos, sizeof(_portalGpsPos), "%.4f, %.4f",
                 gps.location.lat(), gps.location.lng());
      } else {
        snprintf(_portalGpsPos, sizeof(_portalGpsPos), "No fix");
      }
      
      snprintf(_portalGpsSats, sizeof(_portalGpsSats), "%d", gps.satellites.value());
    }
    
    delay(5);
  }

  _cfgServer.stop();
  WiFi.softAPdisconnect(true);
  delay(200);
  restartCleanup(false);
}

/**
 * @brief Returns true when a valid, recent GPS fix is available.
 *
 * "Recent" is defined as a location fix whose age is less than 5 seconds,
 * which guards against stale data being used after signal loss.
 *
 * @return true   gps.location is valid and was updated within the last 5 s.
 * @return false  No fix or fix is too old.
 */
bool hasGpsFix()
{
  return gps.location.isValid() && gps.location.age() < 5000;
}

/**
 * @brief Draws (or erases) a small satellite icon in the top-right corner of
 *        the display to indicate GPS fix status.
 *
 * The icon is always redrawn by first filling its bounding box with black.
 * If hasGpsFix() returns false the function returns immediately, leaving the
 * area blank (no-fix indication).  Otherwise a green satellite silhouette is
 * drawn using primitive LCD calls.
 *
 * Call this after every fillScreen() or any partial redraw that covers the
 * top-right corner so the icon persists across screen updates.
 */
void drawGpsFixIcon()
{
  const int16_t iconX = M5.Lcd.width() - 22;
  const int16_t iconY = 2;
  const int16_t iconW = 20;
  const int16_t iconH = 16;

  M5.Lcd.fillRect(iconX, iconY, iconW, iconH, TFT_BLACK);

  if (!hasGpsFix())
  {
    return;
  }

  const uint16_t iconColor = TFT_GREEN;
  const int16_t bodyX = iconX + 8;
  const int16_t bodyY = iconY + 6;

  M5.Lcd.drawRect(bodyX, bodyY, 4, 4, iconColor);
  M5.Lcd.drawLine(bodyX - 3, bodyY - 2, bodyX, bodyY + 1, iconColor);
  M5.Lcd.drawLine(bodyX + 4, bodyY + 1, bodyX + 7, bodyY - 2, iconColor);
  M5.Lcd.drawLine(bodyX - 4, bodyY - 3, bodyX - 1, bodyY, iconColor);
  M5.Lcd.drawLine(bodyX + 5, bodyY, bodyX + 8, bodyY - 3, iconColor);
  M5.Lcd.drawLine(bodyX + 1, bodyY + 4, bodyX - 2, bodyY + 7, iconColor);
  M5.Lcd.drawLine(bodyX + 3, bodyY + 4, bodyX + 6, bodyY + 7, iconColor);
  M5.Lcd.drawPixel(bodyX - 5, bodyY - 4, iconColor);
  M5.Lcd.drawPixel(bodyX + 9, bodyY - 4, iconColor);
  M5.Lcd.drawCircle(bodyX + 10, bodyY - 4, 2, iconColor);
}

/**
 * @brief Returns a human-readable label for a SoftwareSerial frame format.
 *
 * Used only for diagnostic output during GPS auto-detection.
 *
 * @param config  One of the SWSERIAL_8x1/8x2 constants.
 * @return        Short string such as "8N1", "8E1", etc., or "UNK".
 */
const char* gpsConfigName(EspSoftwareSerial::Config config)
{
  switch (config)
  {
    case SWSERIAL_8N1: return "8N1";
    case SWSERIAL_8E1: return "8E1";
    case SWSERIAL_8O1: return "8O1";
    case SWSERIAL_8N2: return "8N2";
    default: return "UNK";
  }
}

/**
 * @brief Returns true if the null-terminated string looks like a valid NMEA
 *        sentence from a supported constellation.
 *
 * Checks for the leading '$' and a two-letter talker ID from the set:
 *   GP (GPS), GN (combined), GL (GLONASS), GA (Galileo), BD/GB (BeiDou).
 *
 * @param sentence  Null-terminated candidate string (no CR/LF).
 * @return true     Recognised NMEA talker prefix.
 * @return false    Not an NMEA sentence.
 */
bool looksLikeNmeaSentence(const char* sentence)
{
  if (sentence[0] != '$')
  {
    return false;
  }

  return strncmp(sentence, "$GP", 3) == 0 ||
         strncmp(sentence, "$GN", 3) == 0 ||
         strncmp(sentence, "$GL", 3) == 0 ||
         strncmp(sentence, "$GA", 3) == 0 ||
         strncmp(sentence, "$BD", 3) == 0 ||
         strncmp(sentence, "$GB", 3) == 0;
}

/**
 * @brief Tests a single baud-rate / frame-format combination for valid NMEA
 *        output from the GPS module.
 *
 * Reconfigures the SoftwareSerial port, flushes any buffered data, then reads
 * incoming characters for up to @p timeoutMs milliseconds, assembling them
 * into line-terminated sentences.  Returns true as soon as a sentence with a
 * recognised NMEA prefix is found.
 *
 * @param baud       Baud rate to test (e.g. 9600, 38400).
 * @param config     Frame format (e.g. SWSERIAL_8N1).
 * @param timeoutMs  Maximum time to wait for a valid sentence (milliseconds).
 * @param rxPin      GPIO used as SoftwareSerial RX.
 * @param txPin      GPIO used as SoftwareSerial TX.
 * @return true      A valid NMEA sentence was received on this setting.
 * @return false     No valid sentence within the timeout window.
 */
bool tryGpsSerialSetting(uint32_t baud, EspSoftwareSerial::Config config, unsigned long timeoutMs,
                         int rxPin, int txPin)
{
  ss.end();
  delay(25);
  ss.begin(baud, config, rxPin, txPin, false);

  while (ss.available())
  {
    ss.read();
  }

  char sentence[128];
  size_t sentenceIndex = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs)
  {
    while (ss.available())
    {
      char ch = static_cast<char>(ss.read());

      if (ch == '\r')
      {
        continue;
      }

      if (ch == '\n')
      {
        sentence[sentenceIndex] = '\0';
        if (sentenceIndex > 6 && looksLikeNmeaSentence(sentence))
        {
          return true;
        }
        sentenceIndex = 0;
        continue;
      }

      if (ch >= 32 && ch <= 126 && sentenceIndex < sizeof(sentence) - 1)
      {
        sentence[sentenceIndex++] = ch;
      }
      else if (sentenceIndex >= sizeof(sentence) - 1)
      {
        sentenceIndex = 0;
      }
    }

    delay(2);
  }

  return false;
}

/**
 * @brief Auto-detects the GPS module's serial parameters by scanning a matrix
 *        of baud rates and frame formats, trying the stored configuration first
 *        and testing both normal and swapped RX/TX pin mappings.
 *
 * If gpsBaudStr and gpsConfigStr are non-empty (from a prior successful scan),
 * tryGpsSerialSetting() is called with those values first. If that succeeds or
 * those fields are empty, the function iterates through 6 baud rates × 4 frame
 * formats (24 combinations). On the first successful combination the detected
 * values are stored in detectedGpsBaud / detectedGpsConfig and the serial port
 * is left configured for normal operation.
 *
 * If no combination produces a valid NMEA sentence the port falls back to
 * GPSBaud / SWSERIAL_8N1 so subsequent reads can still attempt communication.
 *
 * @return true   A matching configuration was found.
 * @return false  Fallback used; GPS communication may be unreliable.
 */
bool initGpsSerial()
{
  static constexpr uint32_t gpsBaudRates[] = {9600, 38400, 115200, 19200, 57600, 4800};
  static constexpr EspSoftwareSerial::Config gpsConfigs[] = {
    SWSERIAL_8N1,
    SWSERIAL_8E1,
    SWSERIAL_8O1,
    SWSERIAL_8N2,
  };
  struct PinMapping { int rxPin; int txPin; const char* label; };
  static const PinMapping pinMappings[] = {
    {RXPin, TXPin, "33/32"},
    {TXPin, RXPin, "32/33"},
  };
  const PinMapping* pinOrder[] = {&pinMappings[0], &pinMappings[1]};
  if (strcmp(gpsRxPinStr, "32") == 0 && strcmp(gpsTxPinStr, "33") == 0)
  {
    pinOrder[0] = &pinMappings[1];
    pinOrder[1] = &pinMappings[0];
  }

  M5.Lcd.println("Scanning GPS...");

  // Try stored configuration first if available
  if (gpsBaudStr[0] != '\0' && gpsConfigStr[0] != '\0') {
    uint32_t storedBaud = atol(gpsBaudStr);
    EspSoftwareSerial::Config storedConfig = SWSERIAL_8N1;
    if (strcmp(gpsConfigStr, "8E1") == 0) storedConfig = SWSERIAL_8E1;
    else if (strcmp(gpsConfigStr, "8O1") == 0) storedConfig = SWSERIAL_8O1;
    else if (strcmp(gpsConfigStr, "8N2") == 0) storedConfig = SWSERIAL_8N2;
    
    for (const PinMapping* mappingPtr : pinOrder)
    {
      const PinMapping& mapping = *mappingPtr;
      M5.Lcd.printf("GPS stored %lu %s %s\n", storedBaud, gpsConfigStr, mapping.label);
      if (tryGpsSerialSetting(storedBaud, storedConfig, GPSDetectTimeoutMs,
                              mapping.rxPin, mapping.txPin)) {
        detectedGpsBaud = storedBaud;
        detectedGpsConfig = storedConfig;
        activeGpsRxPin = mapping.rxPin;
        activeGpsTxPin = mapping.txPin;
        persistDetectedGpsSettings();
        M5.Lcd.printf("GPS OK %lu %s %s\n", storedBaud, gpsConfigStr, mapping.label);
        return true;
      }
    }
  }

  for (const PinMapping* mappingPtr : pinOrder)
  {
    const PinMapping& mapping = *mappingPtr;
    for (uint32_t baud : gpsBaudRates)
    {
      for (EspSoftwareSerial::Config config : gpsConfigs)
      {
        M5.Lcd.printf("GPS %lu %s %s\n", static_cast<unsigned long>(baud), gpsConfigName(config), mapping.label);

        if (tryGpsSerialSetting(baud, config, GPSDetectTimeoutMs, mapping.rxPin, mapping.txPin))
        {
          detectedGpsBaud = baud;
          detectedGpsConfig = config;
          activeGpsRxPin = mapping.rxPin;
          activeGpsTxPin = mapping.txPin;
          persistDetectedGpsSettings();
          M5.Lcd.printf("GPS OK %lu %s %s\n", static_cast<unsigned long>(baud), gpsConfigName(config), mapping.label);
          return true;
        }
      }
    }
  }

  ss.end();
  delay(25);
  activeGpsRxPin = RXPin;
  activeGpsTxPin = TXPin;
  ss.begin(GPSBaud, SWSERIAL_8N1, activeGpsRxPin, activeGpsTxPin, false);
  detectedGpsBaud = GPSBaud;
  detectedGpsConfig = SWSERIAL_8N1;
  M5.Lcd.println("GPS fallback 9600 8N1");
  return false;
}

/**
 * @brief Arduino setup() – runs once at every boot.
 *
 * Initialises hardware, loads persisted configuration, reads the boot-mode
 * counter from NVS, and dispatches to the appropriate mode handler:
 *
 *   Mode 0  – Initialises the GPS serial port; normal GPS loop continues in
 *              loop().
 *   Mode 1  – Connects to WiFi/MQTT with limited retries and enters closeDoor().
 *   Mode 2  – Connects to WiFi/MQTT with infinite retries and enters openDoor().
 *
 * Modes 1 and 2 never return from setup(); the device reboots or powers off
 * at the end of their respective sequences.
 */
void setup()
{
  M5.begin();
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextFont(2);
  M5.Imu.Init();
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  digitalWrite(Relay1, LOW); // Relay off; mode 0 loop controls it dynamically

  // If BtnA held at boot, or no saved config, launch the setup portal
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  M5.update();
  if (M5.BtnA.isPressed() || !loadConfig())
  {
    runConfigPortal(); // never returns – reboots after save
  }

  preferences.begin("lastAction", false);

  value = preferences.getUInt("counter", 0);
  away  = preferences.getBool("away", false);
  mode0SkipWaitOnBoot = preferences.getBool("m0skipwait", false);

  if (value == 0)
  {
    // ── Mode 0: GPS-only – scan for module then enter loop() ──────────────
    M5.Lcd.println("Mode 0: GPS");
    initGpsSerial();
    // If we just transitioned from mode 1 (away=true), wait 5 minutes before
    // mode-switch decisions. A mode0 USB-loss resume bypasses this once.
    mode0ApproachGateActive = away;
    if (mode0SkipWaitOnBoot)
    {
      mode0ApproachGateActive = false;
      preferences.putBool("m0skipwait", false);
      mode0SkipWaitOnBoot = false;
    }
    mode0WindowInitialized = false;
    mode0WindowStartMs = 0;
    mode0WindowStartDistance = 0.0;
    mode0UsbWasPresent = isUsbPowerPresent();
    drawGpsFixIcon();
    // loop() takes over from here
  }
  else if (value > 0 && value < 100)
  {
    // ── Mode 1: Close door (GPS not used) ────────────────────────────────
    M5.Lcd.println("Mode 1: Close door");
    drawGpsFixIcon();
    setupWifi(false);  // limited retry
    closeDoor();       // blocks until door confirmed closed, then reboots into mode 0
  }
  else if (value >= 100)
  {
    // ── Mode 2: Open door (GPS not used) ─────────────────────────────────
    M5.Lcd.println("Mode 2: Open door");
    drawGpsFixIcon();
    setupWifi(true);  // retry forever
    openDoor();       // blocks until door confirmed open, then powers off into mode 1
  }
}

/**
 * @brief Connects to the configured WiFi network and completes network
 *        initialisation (NTP sync, RTC set, MQTT broker and callback).
 *
 * @param infiniteRetry  When true the function retries WiFi indefinitely
 *                       (used in Mode 2 where the door must be opened).
 *                       When false the device reboots after ~10 failed
 *                       attempts (~40 s) to handle out-of-range scenarios.
 */
void setupWifi(bool infiniteRetry)
{
  WiFi.begin(wifiSsid, wifiPass);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(4000);
    M5.update();
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.printf("WiFi %d status=%d\n", retries, WiFi.status());
    drawGpsFixIcon();
    if (!infiniteRetry && ++retries >= 10) restartCleanup(false);
  }
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.println("WiFi OK");
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  drawGpsFixIcon();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  delay(500);
  ntpGetLocalTime();
  delay(500);
  client.setServer(mqttServer, 1883);
  delay(500);
  client.setCallback(callback);
}

/**
 * @brief Synchronises the M5StickC Plus RTC from the SNTP time source.
 *
 * Calls getLocalTime() in a retry loop.  If the NTP server cannot be reached
 * within 2 attempts the device sets the away flag, stores Mode 0, and
 * reboots rather than operating with an incorrect clock.
 *
 * Requires configTime() to have been called beforehand (done in setupWifi()).
 */
void ntpGetLocalTime()
{
  struct tm       timeinfo;
  RTC_TimeTypeDef TimeStruct;
  RTC_DateTypeDef DateStruct;
  int retries = 0;
  while (!getLocalTime(&timeinfo))
  {
    M5.Lcd.println("NTP failed, retrying...");
    drawGpsFixIcon();
    if (++retries >= 2)
    {
      away = true;
      awayCheck(true);
      resetCounter(0);
      restartCleanup(false);
    }
  }
  TimeStruct.Hours   = timeinfo.tm_hour;
  TimeStruct.Minutes = timeinfo.tm_min;
  TimeStruct.Seconds = timeinfo.tm_sec;
  M5.Rtc.SetTime(&TimeStruct);
  DateStruct.Month = timeinfo.tm_mon + 1;
  DateStruct.Date  = timeinfo.tm_mday;
  DateStruct.Year  = timeinfo.tm_year + 1900;
  M5.Rtc.SetData(&DateStruct);
}

/**
 * @brief PubSubClient MQTT message callback.
 *
 * Invoked automatically by client.loop() whenever a message arrives on a
 * subscribed topic.  Parses the first 4 bytes of the payload to distinguish
 * "open" from "clos" and updates the doorOpen/doorClos flags accordingly.
 * toggleDoor() polls these flags to determine when its command was accepted.
 *
 * @param topic    Topic string of the incoming message.
 * @param payload  Raw bytes of the message payload (NOT null-terminated).
 * @param length   Length of @p payload in bytes.
 */
void callback(char* topic, byte* payload, unsigned int length)
{
  if (length < 4) return;
  if (strncmp((char*)payload, "open", 4) == 0) {
    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    M5.Lcd.println("Door: open");
    drawGpsFixIcon();
    doorOpen = true;
    doorClos = false;
  } else if (strncmp((char*)payload, "clos", 4) == 0) {
    M5.Lcd.setTextColor(TFT_BLUE, TFT_BLACK);
    M5.Lcd.println("Door: closed");
    drawGpsFixIcon();
    doorClos = true;
    doorOpen = false;
  }
}



/**
 * @brief Ensures the MQTT client is connected, reconnecting as necessary.
 *
 * Uses a randomised client ID to avoid broker-side "client already connected"
 * rejections across reboots.  The LWT ("Offline", retained) is registered with
 * the broker during connect() so the broker publishes it automatically if the
 * TCP connection drops without a clean disconnect.  On a successful connection,
 * "Online" is published as a retained message to the same topic so any
 * subscriber (including late-joiners) sees the correct current presence state.
 */
void reConnect()
{
  while (!client.connected()) {
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Lcd.print("MQTT connecting...");
    drawGpsFixIcon();
    char clientId[24];
    snprintf(clientId, sizeof(clientId), "M5Stack-%04X", (unsigned)random(0xffff));
    // Register the LWT with the broker at connect time (QoS 0, retained).
    // "Offline" will be published automatically if the device disconnects
    // without calling client.disconnect() (e.g. power loss, crash, TCP drop).
    if (client.connect(clientId, nullptr, nullptr,
                       topicLWT, 0, /*retain=*/true, "Offline")) {
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
      M5.Lcd.println("OK");
      drawGpsFixIcon();
      // Publish "Online" retained so late-joining subscribers see current state.
      client.publish(topicLWT, "Online", /*retain=*/true);
      client.subscribe(mqttGarageSub);
    } else {
      delay(250);
    }
  }
}

double distanceToHome(double lat, double lon)
{
  static double cachedLat = 0.0, cachedLon = 0.0;
  if (cachedLat == 0.0) {
    cachedLat = atof(homeLat[0] ? homeLat : "37.2431"); // Default to Area 51 if not set
    cachedLon = atof(homeLon[0] ? homeLon : "-115.7930");
  }
  return TinyGPSPlus::distanceBetween(lat, lon, cachedLat, cachedLon);
}

/**
 * @brief Publishes the arrival/departure message and the door command, then
 *        polls until the MQTT callback confirms the door reached the target
 *        state.
 *
 * The loop retries every 1.5 s until @p doorFlag is set to true by the
 * callback.  The MQTT connection is established here if it has dropped.
 *
 * @param arrivedMsg  Payload for the arrival topic (e.g. "Home" or "Gone").
 * @param cmdMsg      Payload for the garage command topic ("open" or "clos").
 * @param doorFlag    Reference to doorOpen or doorClos; set by callback().
 */
void toggleDoor(const char* arrivedMsg, const char* cmdMsg, bool& doorFlag)
{
  if (!client.connected()) { reConnect(); }
  while (!doorFlag)
  {
    client.publish(topicArrived, arrivedMsg);
    delay(500);
    client.publish(mqttGaragePub, cmdMsg);
    delay(1000);
    client.loop();
  }
}

/**
 * @brief Mode 2 terminal action: opens the garage door and powers off.
 *
 * Calls toggleDoor() which blocks until the MQTT "open" confirmation arrives,
 * then persists Mode 1 (counter = 10) so the next wake-up will close the door,
 * clears the away flag, and powers the device off via the AXP192 PMIC.
 */
void openDoor()
{
  toggleDoor("Home", "open", doorOpen);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.println("Door is open");
  drawGpsFixIcon();
  away = false;
  resetCounter(10);
  delay(200);
  restartCleanup(true);
}

/**
 * @brief Mode 1 terminal action: waits for a departure trigger, closes the
 *        garage door, and reboots into Mode 0.
 *
 * Blocks in a polling loop waiting for either:
 *   - BtnA press (manual departure confirmation), or
 *   - Any IMU axis exceeding 1.5 g (device movement / motorcycle motion).
 *
 * Once triggered, publishes "Gone" + "clos" via toggleDoor(), sets the away
 * flag, persists Mode 0, and reboots with a soft reset.
 */
void closeDoor()
{
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextFont(2);
  M5.Lcd.println("Waiting to close door...");
  M5.Lcd.println("Press BtnA or move device.");
  drawGpsFixIcon();

  // Wait for BtnA press or IMU threshold
  while (true) {
    M5.update();
    drawGpsFixIcon();
    float accX, accY, accZ;
    M5.Imu.getAccelData(&accX, &accY, &accZ);

    if (M5.BtnA.wasPressed()) {
      M5.Lcd.println("BtnA: closing...");
      drawGpsFixIcon();
      break;
    }
    if (fabsf(accX) > 1.5f || fabsf(accY) > 1.5f || fabsf(accZ) > 1.5f) {
      M5.Lcd.println("IMU: closing...");
      drawGpsFixIcon();
      break;
    }

    delay(100); // Small delay to avoid busy looping
  }

  // Proceed to close the door
  toggleDoor("Gone", "clos", doorClos);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.println("Door is closed");
  drawGpsFixIcon();
  away = true;
  awayCheck(true);
  resetCounter(0);
  delay(200);
  restartCleanup(false);
}

/**
 * @brief Closes the Preferences handle, waits 1 s for NVS to flush, then
 *        either powers off (powerOff = true) or performs a soft reset.
 *
 * @param powerOff  true  = AXP192 hard power-off (wake on USB/barrel).
 *                  false = ESP.restart() soft reboot (keeps memory powered).
 */
void restartCleanup(bool powerOff)
{
  preferences.end();
  delay(1000);
  powerOff ? M5.Axp.PowerOff() : ESP.restart();
}

/** @brief Writes the boot-mode counter to the "lastAction" NVS namespace. */
void resetCounter(int x)  { preferences.putUInt("counter", x); }

/** @brief Writes the away flag to the "lastAction" NVS namespace. */
void awayCheck(bool x)    { preferences.putBool("away", x); }

/** @brief Returns true when USB/VBUS power is present. */
bool isUsbPowerPresent()
{
  return M5.Axp.GetVBusVoltage() > 4.0f;
}

/**
 * @brief Updates the approach-home detection state machine on each GPS fix.
 *
 * Compares currentDistance against lastDistance:
 *   - First call (lastDistance == 0): initialises lastDistance.
 *   - Moving farther away: resets distanceCount and updates lastDistance.
 *   - Moving closer: increments distanceCount and updates lastDistance.
 *
 * When distanceCount reaches approachCountSP inside the distanceSP zone,
 * loop() triggers a Mode 2 reboot to open the garage door.
 */
void handleDistanceChange()
{
  if (lastDistance == 0.0) {
    lastDistance = currentDistance;
  } else if (currentDistance > lastDistance) {
    distanceCount = 0;
    lastDistance  = currentDistance;
  } else if (currentDistance < lastDistance) {
    distanceCount++;
    lastDistance = currentDistance;
  }
}

/**
 * @brief Reads and feeds incoming GPS serial bytes to the TinyGPSPlus parser
 *        for @p ms milliseconds.
 *
 * Should be called regularly from loop() to keep the parser state machine
 * current.  A short call (e.g. 50 ms) per iteration is sufficient for 9600
 * baud modules which emit a new fix sentence once per second.
 *
 * @param ms  Duration to collect data (milliseconds).
 */
void getGPSData(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available()) //GPS available?
    {
      gps.encode(ss.read()); //Encode the last read
    }
  } while (millis() - start < ms);
}

/**
 * @brief Arduino loop() – Mode 0 GPS-only main loop.
 *
 * Only reached when the boot counter is 0 (Mode 0).  Modes 1 and 2 block
 * entirely inside setup() and never return.
 *
 * Each iteration:
 *  1. Reads GPS data for 50 ms and refreshes the fix icon.
 *  2. Updates currentSpeed and currentDistance from the latest fix.
 *  3. Drives Relay1 based on the speed threshold.
 *  4. Runs the approach-home state machine; reboots into Mode 2 on trigger.
 *  5. Handles BtnB short-press (immediate Mode 2) and long-press (Mode 1 +
 *     power-off).
 *  6. Redraws the speed/distance display only when values change.
 */
void loop()
{
  // loop() is only reached in mode 0 (GPS-only). Modes 1 and 2 block in setup().
  M5.update();

  const bool usbPresent = isUsbPowerPresent();
  if (mode0UsbWasPresent && !usbPresent)
  {
    // USB power dropped while in mode 0: persist resume state and power down.
    resetCounter(0);
    awayCheck(away);
    preferences.putBool("m0skipwait", true);
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Lcd.println("USB lost");
    M5.Lcd.println("Saving state...");
    drawGpsFixIcon();
    delay(200);
    restartCleanup(true);
  }
  mode0UsbWasPresent = usbPresent;

  getGPSData(50);
  drawGpsFixIcon();

  // ── GPS readings ──────────────────────────────────────────────────────────
  bool gpsValid = gps.location.isValid();
  if (gpsValid)
  {
    currentDistance = distanceToHome(gps.location.lat(), gps.location.lng());
    currentSpeed    = gps.speed.mph();
  }
  else
  {
    currentSpeed    = 0.0f;
    currentDistance = 0.0;
  }

  // ── Relay1: ON when below speed setpoint ─────────────────────────────────
  digitalWrite(Relay1, (currentSpeed < speedSP) ? HIGH : LOW);

  // ── Approach-home detection ───────────────────────────────────────────────
  if (mode0ApproachGateActive)
  {
    if (!mode0WindowInitialized)
    {
      mode0WindowInitialized = true;
      mode0WindowStartMs = millis();
      mode0WindowStartDistance = currentDistance;
    }
    else if (millis() - mode0WindowStartMs >= approachWindowMs)
    {
      mode0ApproachGateActive = false;
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
      M5.Lcd.println("Approach wait done");
      drawGpsFixIcon();
    }
  }

  handleDistanceChange();
  if (gpsValid && !mode0ApproachGateActive && currentDistance > 0.0 &&
      currentDistance < distanceSP &&
      distanceCount >= approachCountSP)
  {
    // Approaching home and within zone – reboot into mode 2 to open door
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Lcd.println("Approaching home!");
    M5.Lcd.println("Opening door...");
    drawGpsFixIcon();
    delay(800);
    resetCounter(100);
    restartCleanup(false);
  }

  // ── BtnB: short press = open door now; hold >=5 s = mode 1 + power off ───
  static unsigned long btnBDownAt  = 0;
  static bool          btnBHandled = false;

  if (M5.BtnB.isPressed())
  {
    if (btnBDownAt == 0)
    {
      btnBDownAt  = millis();
      btnBHandled = false;
    }
    if (!btnBHandled && (millis() - btnBDownAt >= 5000))
    {
      // Held >=5 s: store mode 1, power off; on next wake will wait to close door
      btnBHandled = true;
      M5.Lcd.fillScreen(TFT_BLACK);
      M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
      M5.Lcd.println("Mode 1 on wake");
      M5.Lcd.println("Powering off...");
      drawGpsFixIcon();
      delay(500);
      resetCounter(10);      // mode 1 value
      restartCleanup(true);  // power off
    }
  }
  else if (btnBDownAt != 0)
  {
    if (!btnBHandled)
    {
      // Short press: reboot into mode 2 to open door immediately
      resetCounter(100);
      restartCleanup(false);
    }
    btnBDownAt  = 0;
    btnBHandled = false;
  }

  // ── Display: speed (large, centred) + distance; redraw only on change ─────
  static float  lastSpeedDisp = -1.0f;
  static double lastDistDisp  = -1.0;

  if (lastSpeedDisp != currentSpeed || lastDistDisp != currentDistance)
  {
    M5.Lcd.fillScreen(TFT_BLACK);

    char speedStr[8];
    snprintf(speedStr, sizeof(speedStr), "%.1f", currentSpeed);

    int16_t cx = M5.Lcd.width()  / 2;
    int16_t cy = M5.Lcd.height() / 2;

    M5.Lcd.setTextFont(7);
    M5.Lcd.setTextColor((currentSpeed < speedSP) ? TFT_GREEN : TFT_RED, TFT_BLACK);
    M5.Lcd.drawCentreString(speedStr, cx, cy - 24, 7);

    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    char distStr[20];
    snprintf(distStr, sizeof(distStr), "%.0f m", currentDistance);
    M5.Lcd.drawCentreString(distStr, cx, cy + 40, 2);

    drawGpsFixIcon();

    lastSpeedDisp = currentSpeed;
    lastDistDisp  = currentDistance;
  }
}
