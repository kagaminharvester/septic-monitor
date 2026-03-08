/**
 * Septic Tank Level Monitor
 * Board: LILYGO TTGO T-Display (ESP32 + ST7789 135x240 TFT)
 * Sensor: JSN-SR04T waterproof ultrasonic (Trigger/Echo)
 *
 * Features:
 *   - Ultrasonic distance measurement → tank level %
 *   - Beautiful TFT gauge on built-in display
 *   - Web GUI with SVG tank visualization
 *   - Scheduled measurements (1x, 2x, 4x daily or manual)
 *   - Demo mode with live continuous scanning
 *   - Configurable tank depth, sensor offset, reading count
 *   - WiFi AP fallback for initial config
 *   - NVS persistent settings
 *   - SSE streaming for live demo mode
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <LittleFS.h>
#include <time.h>

/* ═══════════════════════════════════════════
 *  PIN DEFINITIONS — TTGO T-Display safe pins
 * ═══════════════════════════════════════════ */
static const uint8_t PIN_TRIG = 26;   ///< JSN-SR04T trigger pin
static const uint8_t PIN_ECHO = 25;   ///< JSN-SR04T echo pin
static const uint8_t PIN_BTN_TOP = 0; ///< Top button (BOOT)
static const uint8_t PIN_BTN_BOT = 35;///< Bottom button

/* ═══════════════════════════════════════════
 *  CONSTANTS
 * ═══════════════════════════════════════════ */
static const float SOUND_SPEED_CM_US = 0.0343f; ///< Speed of sound cm/µs at ~20°C
static const unsigned long ECHO_TIMEOUT_US = 60000UL; ///< Max echo wait (60ms ≈ 10m)
static const float MIN_VALID_DIST_CM = 2.0f;   ///< JSN-SR04T minimum range
static const float MAX_VALID_DIST_CM = 600.0f;  ///< JSN-SR04T maximum range
static const int MAX_LOG_ENTRIES = 50;           ///< Circular log size
static const char* NTP_SERVER = "pool.ntp.org";
static const char* AP_SSID = "SepticMonitor";   ///< Fallback AP name
static const char* AP_PASS = "septic1234";       ///< Fallback AP password

/* ═══════════════════════════════════════════
 *  CONFIG STRUCT — persisted in NVS
 * ═══════════════════════════════════════════ */
struct Config {
    char wifi_ssid[33];       ///< WiFi SSID
    char wifi_pass[65];       ///< WiFi password
    float tank_depth_cm;      ///< Internal tank depth in cm
    float sensor_offset_cm;   ///< Distance from sensor to tank top edge
    int num_readings;         ///< Readings averaged per measurement
    int schedule;             ///< 0=manual, 1=daily, 2=2x, 4=4x
    int measure_duration_sec; ///< How long to collect readings
};

static Config cfg = {
    "", "", 200.0f, 5.0f, 30, 2, 30
};

/* ═══════════════════════════════════════════
 *  LOG ENTRY
 * ═══════════════════════════════════════════ */
struct LogEntry {
    char timestamp[20]; ///< "HH:MM:SS" or "MM/DD HH:MM"
    char message[80];   ///< Log text
};

/* ═══════════════════════════════════════════
 *  GLOBAL STATE
 * ═══════════════════════════════════════════ */
static TFT_eSPI tft = TFT_eSPI();
static AsyncWebServer server(80);
static AsyncEventSource events("/api/stream");
static Preferences prefs;

static float g_distance_cm = 0.0f;   ///< Last measured distance from sensor
static float g_level_pct = 0.0f;     ///< Tank fill percentage
static float g_level_cm = 0.0f;      ///< Liquid height in cm
static bool g_demo_mode = false;     ///< Demo (continuous live scan) active
static bool g_scanning = false;      ///< Currently taking a measurement
static bool g_wifi_connected = false;
static bool g_ap_mode = false;
static char g_last_update[20] = "--";
static unsigned long g_last_scheduled_ms = 0;
static unsigned long g_demo_last_ms = 0;
static volatile bool g_measure_requested = false; ///< Set by web API

static LogEntry g_log[MAX_LOG_ENTRIES];
static int g_log_count = 0;
static int g_log_head = 0; ///< Next write position (circular)

/* ═══════════════════════════════════════════
 *  FORWARD DECLARATIONS
 * ═══════════════════════════════════════════ */
void loadConfig();
void saveConfig();
void addLog(const char* msg);
float measureDistanceSingle();
float measureDistanceAvg(int count, int duration_sec);
void updateLevel(float distance_cm);
void drawTFT();
void setupWiFi();
void setupWebServer();
void handleSchedule();
void sendSSEUpdate();
String getTimeStr();

/* ═══════════════════════════════════════════
 *  NVS CONFIG PERSISTENCE
 * ═══════════════════════════════════════════ */

/**
 * Load configuration from NVS flash.
 * Falls back to defaults if no saved config exists.
 */
void loadConfig() {
    prefs.begin("septic", true);
    strlcpy(cfg.wifi_ssid, prefs.getString("ssid", "").c_str(), sizeof(cfg.wifi_ssid));
    strlcpy(cfg.wifi_pass, prefs.getString("pass", "").c_str(), sizeof(cfg.wifi_pass));
    cfg.tank_depth_cm = prefs.getFloat("depth", 200.0f);
    cfg.sensor_offset_cm = prefs.getFloat("offset", 5.0f);
    cfg.num_readings = prefs.getInt("readings", 30);
    cfg.schedule = prefs.getInt("schedule", 2);
    cfg.measure_duration_sec = prefs.getInt("duration", 30);
    prefs.end();
}

/**
 * Save current configuration to NVS flash.
 */
void saveConfig() {
    prefs.begin("septic", false);
    prefs.putString("ssid", cfg.wifi_ssid);
    prefs.putString("pass", cfg.wifi_pass);
    prefs.putFloat("depth", cfg.tank_depth_cm);
    prefs.putFloat("offset", cfg.sensor_offset_cm);
    prefs.putInt("readings", cfg.num_readings);
    prefs.putInt("schedule", cfg.schedule);
    prefs.putInt("duration", cfg.measure_duration_sec);
    prefs.end();
}

/* ═══════════════════════════════════════════
 *  LOGGING
 * ═══════════════════════════════════════════ */

/**
 * Add a message to the circular log buffer.
 * @param msg The log message text.
 */
void addLog(const char* msg) {
    LogEntry& entry = g_log[g_log_head];
    strlcpy(entry.timestamp, getTimeStr().c_str(), sizeof(entry.timestamp));
    strlcpy(entry.message, msg, sizeof(entry.message));
    g_log_head = (g_log_head + 1) % MAX_LOG_ENTRIES;
    if (g_log_count < MAX_LOG_ENTRIES) g_log_count++;
    log_i("LOG: %s", msg);
}

/**
 * Get current time as formatted string.
 * @return "HH:MM:SS" if NTP synced, else "uptime Xs".
 */
String getTimeStr() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        char buf[20];
        strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
        return String(buf);
    }
    unsigned long s = millis() / 1000;
    return String(s) + "s";
}

/* ═══════════════════════════════════════════
 *  ULTRASONIC MEASUREMENT
 * ═══════════════════════════════════════════ */

/**
 * Take a single distance reading from JSN-SR04T.
 * @return Distance in cm, or -1.0 if timeout/invalid.
 */
float measureDistanceSingle() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    unsigned long duration = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);

    if (duration == 0) {
        return -1.0f;
    }

    float distance = (duration * SOUND_SPEED_CM_US) / 2.0f;

    if (distance < MIN_VALID_DIST_CM || distance > MAX_VALID_DIST_CM) {
        return -1.0f;
    }

    return distance;
}

/**
 * Take multiple readings over a time period and return the median.
 * Outliers are rejected. More readings = higher precision.
 *
 * @param count Target number of valid readings to collect.
 * @param duration_sec Maximum time to spend collecting readings.
 * @return Median distance in cm, or -1.0 if insufficient valid readings.
 */
float measureDistanceAvg(int count, int duration_sec) {
    g_scanning = true;
    float readings[200];
    int valid = 0;
    unsigned long start = millis();
    unsigned long timeout = (unsigned long)duration_sec * 1000UL;
    int attempts = 0;
    int max_attempts = count * 3;

    addLog("Scanning...");

    while (valid < count && (millis() - start) < timeout && attempts < max_attempts) {
        float d = measureDistanceSingle();
        if (d > 0) {
            if (valid < 200) {
                readings[valid++] = d;
            }
        }
        attempts++;
        delay(60); // JSN-SR04T needs ~60ms between triggers
    }

    g_scanning = false;

    if (valid < 3) {
        char buf[80];
        snprintf(buf, sizeof(buf), "Scan fail: only %d/%d valid readings", valid, count);
        addLog(buf);
        return -1.0f;
    }

    // Sort for median
    for (int i = 0; i < valid - 1; i++) {
        for (int j = i + 1; j < valid; j++) {
            if (readings[j] < readings[i]) {
                float tmp = readings[i];
                readings[i] = readings[j];
                readings[j] = tmp;
            }
        }
    }

    // Reject outliers: use middle 60% of sorted readings
    int trim = valid / 5;
    float sum = 0;
    int trimmed_count = 0;
    for (int i = trim; i < valid - trim; i++) {
        sum += readings[i];
        trimmed_count++;
    }

    if (trimmed_count == 0) return -1.0f;

    float median = sum / trimmed_count;

    char buf[80];
    snprintf(buf, sizeof(buf), "Scan OK: %.1f cm (%d readings, %d valid)", median, attempts, valid);
    addLog(buf);

    return median;
}

/**
 * Convert measured distance to tank level percentage.
 * @param distance_cm Distance from sensor to liquid surface.
 */
void updateLevel(float distance_cm) {
    if (distance_cm < 0) return;

    // Subtract sensor offset (distance from sensor to tank top edge)
    float effective_dist = distance_cm - cfg.sensor_offset_cm;
    if (effective_dist < 0) effective_dist = 0;

    g_distance_cm = distance_cm;
    g_level_cm = cfg.tank_depth_cm - effective_dist;
    if (g_level_cm < 0) g_level_cm = 0;
    if (g_level_cm > cfg.tank_depth_cm) g_level_cm = cfg.tank_depth_cm;

    g_level_pct = (g_level_cm / cfg.tank_depth_cm) * 100.0f;

    strlcpy(g_last_update, getTimeStr().c_str(), sizeof(g_last_update));
}

/* ═══════════════════════════════════════════
 *  TFT DISPLAY — Tank gauge on 135x240 screen
 * ═══════════════════════════════════════════ */

/**
 * Draw the tank level gauge on the TFT display.
 * Shows: fill bar, percentage, distance, status.
 */
void drawTFT() {
    tft.fillScreen(TFT_BLACK);

    // Title
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.setTextFont(2);
    tft.drawString("SEPTIC TANK", 67, 4);

    // Tank outline
    int tank_x = 20, tank_y = 30, tank_w = 95, tank_h = 160;
    tft.drawRoundRect(tank_x, tank_y, tank_w, tank_h, 6, TFT_DARKGREY);
    tft.drawRoundRect(tank_x + 1, tank_y + 1, tank_w - 2, tank_h - 2, 5, 0x4208);

    // Lid
    tft.fillRoundRect(tank_x - 4, tank_y - 4, tank_w + 8, 8, 3, TFT_DARKGREY);

    // Fill bar
    int fill_h = (int)((g_level_pct / 100.0f) * (tank_h - 6));
    if (fill_h < 0) fill_h = 0;
    if (fill_h > tank_h - 6) fill_h = tank_h - 6;

    uint16_t fill_color;
    if (g_level_pct > 80) fill_color = TFT_RED;
    else if (g_level_pct > 60) fill_color = TFT_ORANGE;
    else if (g_level_pct > 40) fill_color = TFT_YELLOW;
    else fill_color = TFT_CYAN;

    int fill_y = tank_y + 3 + (tank_h - 6) - fill_h;
    if (fill_h > 0) {
        tft.fillRoundRect(tank_x + 3, fill_y, tank_w - 6, fill_h, 3, fill_color);
        // Wave line
        for (int wx = tank_x + 3; wx < tank_x + tank_w - 3; wx += 2) {
            int wy = fill_y + (int)(2.0f * sin((wx + millis() / 200.0f) * 0.15f));
            tft.drawPixel(wx, wy, TFT_WHITE);
        }
    }

    // Scale markers
    for (int pct = 25; pct <= 75; pct += 25) {
        int my = tank_y + 3 + (int)((1.0f - pct / 100.0f) * (tank_h - 6));
        tft.drawLine(tank_x - 2, my, tank_x + 2, my, TFT_DARKGREY);
    }

    // Percentage (big, right side)
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(fill_color, TFT_BLACK);
    tft.setTextFont(7);
    // Format: show integer if >= 10, else one decimal
    char pctBuf[8];
    if (g_level_pct >= 10.0f) {
        snprintf(pctBuf, sizeof(pctBuf), "%d", (int)g_level_pct);
    } else {
        snprintf(pctBuf, sizeof(pctBuf), "%.1f", g_level_pct);
    }
    tft.drawString(pctBuf, 132, 36);
    tft.setTextFont(2);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("% FULL", 132, 88);

    // Stats (right side, below percentage)
    tft.setTextFont(1);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    char statBuf[24];
    snprintf(statBuf, sizeof(statBuf), "%.0f cm", g_distance_cm);
    tft.drawString("DIST:", 132, 108);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString(statBuf, 132, 120);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    snprintf(statBuf, sizeof(statBuf), "%.0f cm", g_level_cm);
    tft.drawString("LEVEL:", 132, 136);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString(statBuf, 132, 148);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    snprintf(statBuf, sizeof(statBuf), "%.0f cm", cfg.tank_depth_cm);
    tft.drawString("DEPTH:", 132, 164);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString(statBuf, 132, 176);

    // Status bar (bottom)
    tft.setTextDatum(BL_DATUM);
    tft.setTextFont(1);
    if (g_demo_mode) {
        tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
        tft.drawString("DEMO LIVE", 4, 236);
    } else if (g_scanning) {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawString("SCANNING...", 4, 236);
    } else {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString("IDLE", 4, 236);
    }

    tft.setTextDatum(BR_DATUM);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    if (g_wifi_connected) {
        tft.drawString(WiFi.localIP().toString().c_str(), 132, 236);
    } else if (g_ap_mode) {
        tft.drawString("AP:SepticMonitor", 132, 236);
    } else {
        tft.drawString("No WiFi", 132, 236);
    }

    // Last update time
    tft.setTextDatum(BC_DATUM);
    tft.setTextColor(0x4208, TFT_BLACK);
    tft.drawString(g_last_update, 67, 226);
}

/* ═══════════════════════════════════════════
 *  WiFi SETUP
 * ═══════════════════════════════════════════ */

/**
 * Connect to WiFi. Falls back to AP mode if no credentials
 * or connection fails after 15 seconds.
 */
void setupWiFi() {
    if (strlen(cfg.wifi_ssid) == 0) {
        log_i("No WiFi SSID configured, starting AP mode");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASS);
        g_ap_mode = true;
        addLog("AP mode: SepticMonitor");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);

    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(2);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("Connecting WiFi...", 67, 100);
    tft.setTextFont(1);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString(cfg.wifi_ssid, 67, 130);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
        delay(500);
        tft.drawString(".", 67, 150);
    }

    if (WiFi.status() == WL_CONNECTED) {
        g_wifi_connected = true;
        g_ap_mode = false;
        configTime(0, 0, NTP_SERVER);

        char buf[80];
        snprintf(buf, sizeof(buf), "WiFi OK: %s", WiFi.localIP().toString().c_str());
        addLog(buf);
    } else {
        log_w("WiFi failed, falling back to AP");
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP(AP_SSID, AP_PASS);
        g_ap_mode = true;
        addLog("WiFi fail -> AP mode");
    }
}

/* ═══════════════════════════════════════════
 *  SSE STREAMING (for demo mode live updates)
 * ═══════════════════════════════════════════ */

/**
 * Send current measurement data to all connected SSE clients.
 */
void sendSSEUpdate() {
    if (events.count() == 0) return;

    JsonDocument doc;
    doc["level_pct"] = round(g_level_pct * 10.0f) / 10.0f;
    doc["distance_cm"] = round(g_distance_cm * 10.0f) / 10.0f;
    doc["level_cm"] = round(g_level_cm * 10.0f) / 10.0f;
    doc["tank_depth_cm"] = cfg.tank_depth_cm;

    String json;
    serializeJson(doc, json);
    events.send(json.c_str(), "message", millis());
}

/* ═══════════════════════════════════════════
 *  WEB SERVER + API
 * ═══════════════════════════════════════════ */

// HTML page stored as raw string literal to avoid LittleFS dependency
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Septic Tank Monitor</title>
<style>
:root{--bg:#0f172a;--card:#1e293b;--border:#334155;--text:#e2e8f0;--dim:#94a3b8;--accent:#38bdf8;--green:#22c55e;--yellow:#eab308;--orange:#f97316;--red:#ef4444}
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--text);min-height:100vh}
.c{max-width:480px;margin:0 auto;padding:16px}
h1{font-size:1.3rem;text-align:center;padding:12px 0;color:var(--accent);letter-spacing:1px}
.cd{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:16px;margin-bottom:14px}
.cd h2{font-size:.85rem;color:var(--dim);text-transform:uppercase;letter-spacing:1.5px;margin-bottom:12px}
.tw{display:flex;flex-direction:column;align-items:center}
.ts{display:grid;grid-template-columns:1fr 1fr;gap:8px;width:100%;margin-top:14px}
.st{background:var(--bg);border-radius:8px;padding:10px;text-align:center}
.st .v{font-size:1.5rem;font-weight:700}
.st .l{font-size:.7rem;color:var(--dim);text-transform:uppercase;margin-top:2px}
.fd{margin-bottom:12px}
.fd label{display:block;font-size:.8rem;color:var(--dim);margin-bottom:4px}
.fd input,.fd select{width:100%;padding:8px 10px;background:var(--bg);border:1px solid var(--border);border-radius:6px;color:var(--text);font-size:.95rem}
.fd input:focus,.fd select:focus{outline:none;border-color:var(--accent)}
.rw{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.btn{display:block;width:100%;padding:10px;border:none;border-radius:8px;font-size:.95rem;font-weight:600;cursor:pointer;transition:opacity .2s}
.btn:hover{opacity:.85}
.bp{background:var(--accent);color:#0f172a}
.bd{background:#7c3aed;color:#fff}
.bs{background:var(--green);color:#0f172a}
.lg{max-height:140px;overflow-y:auto;font-family:monospace;font-size:.75rem;color:var(--dim);background:var(--bg);border-radius:6px;padding:8px;line-height:1.5}
.sb{display:flex;justify-content:space-between;align-items:center;font-size:.75rem;color:var(--dim);padding:6px 0}
.dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:4px}
.dot.on{background:var(--green)}.dot.sc{background:var(--accent);animation:p 1s infinite}.dot.off{background:var(--red)}
@keyframes p{0%,100%{opacity:1}50%{opacity:.3}}
</style>
</head>
<body>
<div class="c">
<h1>SEPTIC TANK MONITOR</h1>
<div class="sb"><span><span class="dot" id="sd"></span><span id="st">Connecting...</span></span><span id="lu">--</span></div>
<div class="cd"><h2>Tank Level</h2><div class="tw">
<svg viewBox="0 0 200 320" style="width:100%;max-width:280px" xmlns="http://www.w3.org/2000/svg">
<defs>
<linearGradient id="lg" x1="0" y1="0" x2="0" y2="1"><stop offset="0%" id="gt" stop-color="#38bdf8" stop-opacity=".9"/><stop offset="100%" id="gb" stop-color="#0369a1" stop-opacity=".95"/></linearGradient>
<linearGradient id="tw" x1="0" y1="0" x2="1" y2="0"><stop offset="0%" stop-color="#475569"/><stop offset="50%" stop-color="#64748b"/><stop offset="100%" stop-color="#475569"/></linearGradient>
<filter id="gl"><feGaussianBlur stdDeviation="2" result="b"/><feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter>
<clipPath id="tc"><rect x="30" y="40" width="140" height="240" rx="8"/></clipPath>
</defs>
<rect x="24" y="34" width="152" height="252" rx="12" fill="none" stroke="url(#tw)" stroke-width="4"/>
<rect x="30" y="40" width="140" height="240" rx="8" fill="#0c1425"/>
<g clip-path="url(#tc)">
<rect id="lr" x="30" y="280" width="140" height="0" fill="url(#lg)"/>
<path id="wv" d="" fill="rgba(56,189,248,.15)" filter="url(#gl)"/>
</g>
<polygon points="100,8 94,20 106,20" fill="#38bdf8" opacity=".7"/>
<line x1="100" y1="20" x2="100" y2="38" stroke="#38bdf8" stroke-width="1.5" stroke-dasharray="3,3" opacity=".5"/>
<text x="100" y="6" text-anchor="middle" fill="#94a3b8" font-size="7" font-family="sans-serif">SENSOR</text>
<text id="pt" x="100" y="220" text-anchor="middle" fill="#fff" font-size="32" font-weight="bold" font-family="sans-serif" opacity=".9">--</text>
<text id="pu" x="100" y="238" text-anchor="middle" fill="#fff" font-size="11" font-family="sans-serif" opacity=".5">% FULL</text>
<rect x="18" y="28" width="164" height="10" rx="5" fill="#475569"/>
<rect x="80" y="24" width="40" height="8" rx="4" fill="#64748b"/>
<line x1="22" y1="70" x2="28" y2="70" stroke="#475569"/><text x="18" y="73" text-anchor="end" fill="#475569" font-size="7" font-family="sans-serif">75</text>
<line x1="22" y1="130" x2="28" y2="130" stroke="#475569"/><text x="18" y="133" text-anchor="end" fill="#475569" font-size="7" font-family="sans-serif">50</text>
<line x1="22" y1="190" x2="28" y2="190" stroke="#475569"/><text x="18" y="193" text-anchor="end" fill="#475569" font-size="7" font-family="sans-serif">25</text>
<line x1="10" y1="290" x2="190" y2="290" stroke="#334155" stroke-dasharray="4,4"/>
<text x="190" y="300" text-anchor="end" fill="#334155" font-size="7" font-family="sans-serif">GROUND</text>
</svg>
<div class="ts">
<div class="st"><div class="v" id="vp">--</div><div class="l">% Full</div></div>
<div class="st"><div class="v" id="vd">--</div><div class="l">cm to surface</div></div>
<div class="st"><div class="v" id="vl">--</div><div class="l">cm liquid</div></div>
<div class="st"><div class="v" id="vf">--</div><div class="l">cm free space</div></div>
</div></div></div>
<div class="cd"><h2>Controls</h2>
<div class="rw" style="margin-bottom:10px">
<button class="btn bp" onclick="doMeasure()">Measure Now</button>
<button class="btn bd" id="db" onclick="toggleDemo()">Start Demo</button>
</div></div>
<div class="cd"><h2>Settings</h2>
<div class="rw"><div class="fd"><label>Tank Depth (cm)</label><input type="number" id="sd1" min="30" max="500" value="200"></div>
<div class="fd"><label>Sensor Offset (cm)</label><input type="number" id="so" min="0" max="50" value="5"></div></div>
<div class="rw"><div class="fd"><label>Readings per Measure</label><input type="number" id="nr" min="5" max="200" value="30"></div>
<div class="fd"><label>Schedule</label><select id="sc"><option value="1">Once daily (6AM)</option><option value="2" selected>Twice daily</option><option value="4">Every 6h</option><option value="0">Manual only</option></select></div></div>
<div class="rw"><div class="fd"><label>Measure Duration (sec)</label><input type="number" id="md" min="5" max="120" value="30"></div>
<div class="fd"><label>WiFi SSID</label><input type="text" id="ws" placeholder="Your WiFi"></div></div>
<div class="fd"><label>WiFi Password</label><input type="password" id="wp" placeholder="Password"></div>
<button class="btn bs" onclick="doSave()">Save Settings</button></div>
<div class="cd"><h2>Measurement Log</h2><div class="lg" id="lb">Waiting...</div></div>
</div>
<script>
var dm=false,es=null;
function uTank(p,d,td){
p=Math.max(0,Math.min(100,p));var h=p/100*240,y=280-h;
document.getElementById('lr').setAttribute('y',y);
document.getElementById('lr').setAttribute('height',h);
var tc,bc;
if(p>80){tc='#ef4444';bc='#991b1b'}else if(p>60){tc='#f97316';bc='#9a3412'}else if(p>40){tc='#eab308';bc='#854d0e'}else{tc='#38bdf8';bc='#0369a1'}
document.getElementById('gt').setAttribute('stop-color',tc);
document.getElementById('gb').setAttribute('stop-color',bc);
if(p>2){var w='M30,'+y+' Q65,'+(y-4)+' 100,'+y+' T170,'+y+' L170,'+(y+6)+' Q135,'+(y+2)+' 100,'+(y+6)+' T30,'+(y+6)+' Z';
document.getElementById('wv').setAttribute('d',w)}else{document.getElementById('wv').setAttribute('d','')}
var pt=document.getElementById('pt');pt.textContent=p.toFixed(1);
pt.setAttribute('y',Math.min(260,Math.max(60,y+h/2+8)));
document.getElementById('pu').setAttribute('y',Math.min(275,Math.max(75,y+h/2+24)));
document.getElementById('vp').textContent=p.toFixed(1);
document.getElementById('vd').textContent=d.toFixed(1);
var lv=td-d;if(lv<0)lv=0;
document.getElementById('vl').textContent=lv.toFixed(1);
document.getElementById('vf').textContent=d.toFixed(1)}
async function gS(){try{var r=await fetch('/api/status');var d=await r.json();
uTank(d.level_pct,d.distance_cm,d.tank_depth_cm);
document.getElementById('sd').className='dot '+(d.demo_mode?'sc':(d.scanning?'sc':'on'));
document.getElementById('st').textContent=d.demo_mode?'Demo (live)':(d.scanning?'Scanning...':'Online');
document.getElementById('lu').textContent=d.last_update||'--';
if(d.demo_mode!==dm){dm=d.demo_mode;document.getElementById('db').textContent=dm?'Stop Demo':'Start Demo'}}
catch(e){document.getElementById('sd').className='dot off';document.getElementById('st').textContent='Offline'}}
async function doMeasure(){try{await fetch('/api/measure',{method:'POST'})}catch(e){}setTimeout(gS,2000)}
async function toggleDemo(){dm=!dm;try{await fetch('/api/demo',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:dm})})}catch(e){}
document.getElementById('db').textContent=dm?'Stop Demo':'Start Demo';if(dm)startSSE();else stopSSE()}
function startSSE(){stopSSE();es=new EventSource('/api/stream');es.onmessage=function(e){try{var d=JSON.parse(e.data);uTank(d.level_pct,d.distance_cm,d.tank_depth_cm)}catch(x){}};es.onerror=function(){setTimeout(startSSE,3000)}}
function stopSSE(){if(es){es.close();es=null}}
async function doSave(){var c={tank_depth_cm:parseInt(document.getElementById('sd1').value),sensor_offset_cm:parseInt(document.getElementById('so').value),
num_readings:parseInt(document.getElementById('nr').value),schedule:parseInt(document.getElementById('sc').value),
measure_duration_sec:parseInt(document.getElementById('md').value),wifi_ssid:document.getElementById('ws').value,wifi_pass:document.getElementById('wp').value};
try{var r=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(c)});
var d=await r.json();aL(d.ok?'Settings saved':'Error: '+d.error)}catch(e){aL('Save error')}}
async function lS(){try{var r=await fetch('/api/settings');var d=await r.json();
document.getElementById('sd1').value=d.tank_depth_cm||200;document.getElementById('so').value=d.sensor_offset_cm||5;
document.getElementById('nr').value=d.num_readings||30;document.getElementById('sc').value=d.schedule||2;
document.getElementById('md').value=d.measure_duration_sec||30;document.getElementById('ws').value=d.wifi_ssid||''}catch(e){}}
async function lL(){try{var r=await fetch('/api/log');var d=await r.json();var b=document.getElementById('lb');b.textContent='';
(d.entries||[]).forEach(function(e){aL(e.msg,e.ts)})}catch(e){}}
function aL(m,t){var b=document.getElementById('lb');t=t||new Date().toLocaleTimeString();
var d=document.createElement('div');d.textContent='['+t+'] '+m;b.appendChild(d);b.scrollTop=b.scrollHeight}
lS();gS();lL();setInterval(gS,5000);
</script>
</body></html>
)rawliteral";

/**
 * Setup all web server routes and API endpoints.
 */
void setupWebServer() {
    // Serve main page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/html", INDEX_HTML);
    });

    // GET /api/status — current state
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
        JsonDocument doc;
        doc["level_pct"] = round(g_level_pct * 10.0f) / 10.0f;
        doc["distance_cm"] = round(g_distance_cm * 10.0f) / 10.0f;
        doc["level_cm"] = round(g_level_cm * 10.0f) / 10.0f;
        doc["tank_depth_cm"] = cfg.tank_depth_cm;
        doc["demo_mode"] = g_demo_mode;
        doc["scanning"] = g_scanning;
        doc["last_update"] = g_last_update;

        String json;
        serializeJson(doc, json);
        request->send(200, "application/json", json);
    });

    // POST /api/measure — trigger single measurement via flag
    server.on("/api/measure", HTTP_POST, [](AsyncWebServerRequest* request) {
        g_measure_requested = true;
        request->send(200, "application/json", "{\"ok\":true}");
    });

    // POST /api/demo — toggle demo mode (body: {"enabled":true/false})
    server.on("/api/demo", HTTP_POST, [](AsyncWebServerRequest* request) {},
        NULL,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len) == DeserializationError::Ok) {
                g_demo_mode = doc["enabled"] | false;
                char buf[40];
                snprintf(buf, sizeof(buf), "Demo mode: %s", g_demo_mode ? "ON" : "OFF");
                addLog(buf);
            }
            request->send(200, "application/json", "{\"ok\":true}");
        });

    // GET /api/settings
    server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
        JsonDocument doc;
        doc["tank_depth_cm"] = cfg.tank_depth_cm;
        doc["sensor_offset_cm"] = cfg.sensor_offset_cm;
        doc["num_readings"] = cfg.num_readings;
        doc["schedule"] = cfg.schedule;
        doc["measure_duration_sec"] = cfg.measure_duration_sec;
        doc["wifi_ssid"] = cfg.wifi_ssid;

        String json;
        serializeJson(doc, json);
        request->send(200, "application/json", json);
    });

    // POST /api/settings
    server.on("/api/settings", HTTP_POST, [](AsyncWebServerRequest* request) {},
        NULL,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len) != DeserializationError::Ok) {
                request->send(400, "application/json", "{\"ok\":false,\"error\":\"bad json\"}");
                return;
            }

            if (doc["tank_depth_cm"].is<float>()) cfg.tank_depth_cm = doc["tank_depth_cm"];
            if (doc["sensor_offset_cm"].is<float>()) cfg.sensor_offset_cm = doc["sensor_offset_cm"];
            if (doc["num_readings"].is<int>()) cfg.num_readings = constrain((int)doc["num_readings"], 5, 200);
            if (doc["schedule"].is<int>()) cfg.schedule = constrain((int)doc["schedule"], 0, 4);
            if (doc["measure_duration_sec"].is<int>()) cfg.measure_duration_sec = constrain((int)doc["measure_duration_sec"], 5, 120);

            const char* ssid = doc["wifi_ssid"];
            if (ssid) strlcpy(cfg.wifi_ssid, ssid, sizeof(cfg.wifi_ssid));
            const char* pass = doc["wifi_pass"];
            if (pass && strlen(pass) > 0) strlcpy(cfg.wifi_pass, pass, sizeof(cfg.wifi_pass));

            saveConfig();
            addLog("Settings saved");
            request->send(200, "application/json", "{\"ok\":true}");
        });

    // GET /api/log
    server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest* request) {
        JsonDocument doc;
        JsonArray entries = doc["entries"].to<JsonArray>();
        // Read from oldest to newest
        int start = (g_log_count >= MAX_LOG_ENTRIES) ? g_log_head : 0;
        for (int i = 0; i < g_log_count; i++) {
            int idx = (start + i) % MAX_LOG_ENTRIES;
            JsonObject entry = entries.add<JsonObject>();
            entry["ts"] = g_log[idx].timestamp;
            entry["msg"] = g_log[idx].message;
        }
        String json;
        serializeJson(doc, json);
        request->send(200, "application/json", json);
    });

    // SSE endpoint for demo mode streaming
    events.onConnect([](AsyncEventSourceClient* client) {
        log_i("SSE client connected");
    });
    server.addHandler(&events);

    server.begin();
    addLog("Web server started");
}

/* ═══════════════════════════════════════════
 *  SCHEDULE HANDLER
 * ═══════════════════════════════════════════ */

/**
 * Check if a scheduled measurement is due and trigger it.
 * Schedule options: 0=manual, 1=6AM, 2=6AM+6PM, 4=every 6h.
 */
void handleSchedule() {
    if (cfg.schedule == 0) return;

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 100)) return;

    int hour = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;
    bool shouldMeasure = false;

    // Only trigger in the first 2 minutes of the target hour
    if (minute > 1) return;

    switch (cfg.schedule) {
        case 1: shouldMeasure = (hour == 6); break;
        case 2: shouldMeasure = (hour == 6 || hour == 18); break;
        case 4: shouldMeasure = (hour == 0 || hour == 6 || hour == 12 || hour == 18); break;
    }

    if (!shouldMeasure) return;

    // Prevent double-trigger: minimum 1 hour between scheduled measurements
    if (millis() - g_last_scheduled_ms < 3600000UL) return;

    g_last_scheduled_ms = millis();
    addLog("Scheduled measurement");

    float dist = measureDistanceAvg(cfg.num_readings, cfg.measure_duration_sec);
    if (dist > 0) {
        updateLevel(dist);
        drawTFT();
    }
}

/* ═══════════════════════════════════════════
 *  SETUP
 * ═══════════════════════════════════════════ */

void setup() {
    Serial.begin(115200);
    log_i("Septic Tank Monitor starting...");

    // Ultrasonic pins
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIG, LOW);

    // Buttons
    pinMode(PIN_BTN_TOP, INPUT_PULLUP);
    pinMode(PIN_BTN_BOT, INPUT_PULLUP);

    // TFT init
    tft.init();
    tft.setRotation(1); // Landscape, USB on left
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(2);
    tft.drawString("SEPTIC MONITOR", 120, 50);
    tft.setTextFont(1);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("v1.0 - JSN-SR04T", 120, 80);

    // Load config
    loadConfig();
    addLog("System boot");

    // WiFi
    setupWiFi();

    // Web server
    setupWebServer();

    // Initial reading
    delay(500);
    float dist = measureDistanceSingle();
    if (dist > 0) {
        updateLevel(dist);
        addLog("Initial reading OK");
    }

    // Draw initial display
    drawTFT();
}

/* ═══════════════════════════════════════════
 *  MAIN LOOP
 * ═══════════════════════════════════════════ */

void loop() {
    // Handle web-triggered measurement request
    if (g_measure_requested) {
        g_measure_requested = false;
        addLog("Web: measure requested");
        float dist = measureDistanceAvg(cfg.num_readings, cfg.measure_duration_sec);
        if (dist > 0) {
            updateLevel(dist);
        }
        drawTFT();
    }

    // Button handling: Top button = measure now, Bottom = toggle demo
    static bool btn_top_prev = true, btn_bot_prev = true;
    bool btn_top = digitalRead(PIN_BTN_TOP);
    bool btn_bot = digitalRead(PIN_BTN_BOT);

    if (!btn_top && btn_top_prev) {
        // Top button pressed: single measurement
        addLog("Button: measure");
        float dist = measureDistanceAvg(cfg.num_readings, cfg.measure_duration_sec);
        if (dist > 0) {
            updateLevel(dist);
        }
        drawTFT();
    }

    if (!btn_bot && btn_bot_prev) {
        // Bottom button pressed: toggle demo
        g_demo_mode = !g_demo_mode;
        char buf[30];
        snprintf(buf, sizeof(buf), "Demo: %s", g_demo_mode ? "ON" : "OFF");
        addLog(buf);
        drawTFT();
    }

    btn_top_prev = btn_top;
    btn_bot_prev = btn_bot;

    // Demo mode: continuous scanning
    if (g_demo_mode) {
        if (millis() - g_demo_last_ms > 100) { // ~10 Hz in demo
            g_demo_last_ms = millis();
            float dist = measureDistanceSingle();
            if (dist > 0) {
                updateLevel(dist);
                sendSSEUpdate();
            }
            // Redraw TFT every 500ms in demo to avoid flicker
            static unsigned long tft_last = 0;
            if (millis() - tft_last > 500) {
                tft_last = millis();
                drawTFT();
            }
        }
    }

    // Scheduled measurements
    handleSchedule();

    // WiFi reconnect check every 30s
    static unsigned long wifi_check_ms = 0;
    if (!g_ap_mode && millis() - wifi_check_ms > 30000) {
        wifi_check_ms = millis();
        if (WiFi.status() != WL_CONNECTED) {
            g_wifi_connected = false;
            WiFi.reconnect();
        } else {
            g_wifi_connected = true;
        }
    }

    delay(10);
}
