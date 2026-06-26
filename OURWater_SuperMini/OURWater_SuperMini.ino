// =============================================================================
//  OURWater Super Mini  v1.0.0
//  ESP32-S3 Super Mini — WiFi/MQTT standalone water monitoring node
//
//  Connects to the internet via a USB 4G dongle.
//  The dongle is power-cycled periodically via an NPN transistor on GPIO 5
//  (transistor base HIGH = button pressed to GND = dongle power event).
//
//  Required library: PubSubClient (install via Arduino Library Manager)
// =============================================================================

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "board_config.h"

#define FIRMWARE_VER    "SM-1.0.0"
#define VALVE_TRAVEL_MS  15000    // ~15 s end-to-end travel; relay de-energised by serviceValve()
#define WDT_TIMEOUT_S       30    // generous interim — tightened after dongle cycle is non-blocking

// ─── Pin assignments ──────────────────────────────────────────────────────────
#define FLOW_1_PIN         10
#define VALVE_1_OPEN       11
#define VALVE_1_CLOSE      12
#define SOLAR_VOLTAGE_PIN   3
#define BATTERY_24V_PIN     4
#define DONGLE_BUTTON_PIN   5
#define NEO_PIN            48   // onboard WS2812B RGB LED
#define STATUS_LED_PIN     13   // simple blue LED — kept as secondary indicator
#define BATTERY_SDA         6   // free — was MAX17048, removed
#define BATTERY_SCL         7   // free — was MAX17048, removed

// ─── ADC calibration — 12V LiFePO4 pack + 3× parallel 10W panels ────────────
// true_scale = V_multimeter / (pin_mV / 1000.0)  — see [CAL] print in setup()
// Calibrated 2026-06-25: true=13.14V, pin≈2381mV, scale=13.14/2.381=5.52
#define BATT_ADC_SCALE   5.52f
#define SOLAR_ADC_SCALE  11.0f   // not yet calibrated — verify with multimeter

// 3× 36-cell panels in parallel — Voc ~22V
#define SOLAR_MAX_V    22.0f

// ─── Load-shedding tier thresholds (resting voltage, measured DONGLE-OFF) ────
// NOTE: ADC reads in this firmware are taken with dongle ON — ~0.1-0.2V sag
// compensation may be needed for precision tier decisions.
#define BATT_CONSERVE_V      12.5f   // 15% — shed publish windows, hourly only
#define BATT_CRITICAL_V      12.2f   //  8% — dongle off, status only
#define BATT_VALVE_RESERVE_V 11.8f   //  3% — dongle off, valve only, 1×/day

// ─── Enums — must be before Arduino hoists prototypes ────────────────────────
enum LedMode   : uint8_t { LED_DISCONNECTED, LED_CONNECTED, LED_DONGLING };
enum PowerTier : uint8_t { TIER_NORMAL, TIER_CONSERVE, TIER_CRITICAL, TIER_VALVE_RESERVE };

void setLedMode(LedMode m);   // forward declaration — body below state block

void serviceLED() {
    uint32_t now = millis();
    if (TEST_MODE) {
        neopixelWrite(NEO_PIN, 0, 30, 0);   // solid green — test mode
        return;
    }
    extern LedMode  ledMode;
    extern uint32_t ledToggleMs;
    extern bool     ledOn;
    uint32_t period;
    uint8_t  r = 0, g = 0, b = 0;
    switch (ledMode) {
        case LED_CONNECTED:    period = 1000; b = 30;         break;   // 1s blue blink
        case LED_DONGLING:     period = 3000; r = 25; g = 10; break;   // 3s orange blink
        case LED_DISCONNECTED: period =  250; r = 30;         break;   // fast red blink
        default:               period = 1000;                 break;
    }
    if (now - ledToggleMs >= period) {
        ledToggleMs = now;
        ledOn = !ledOn;
        ledOn ? neopixelWrite(NEO_PIN, r, g, b) : neopixelWrite(NEO_PIN, 0, 0, 0);
    }
}

// ─── CA certificate (DigiCert Global Root G2 — same as OURWater.ino) ─────────
const char* CA_CERT =
"-----BEGIN CERTIFICATE-----\n"
"MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n"
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
"MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n"
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n"
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n"
"2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n"
"1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n"
"q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n"
"tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n"
"vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n"
"BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n"
"5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n"
"1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n"
"NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n"
"Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n"
"8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n"
"pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n"
"MrY=\n"
"-----END CERTIFICATE-----\n";

// ─── Flow ISR ────────────────────────────────────────────────────────────────
RTC_NOINIT_ATTR uint32_t pulseCount;
RTC_NOINIT_ATTR uint32_t pulseCountMagic;
static const uint32_t    PULSE_MAGIC = 0x4F574C57;   // 'OWLW' sentinel — OURWater flow

volatile int64_t lastFlowUs = 0;   // debounce timestamp — plain RAM, reset on every boot is fine

void IRAM_ATTR onFlow1(void* arg) {
    int64_t n = esp_timer_get_time();        // microseconds, IRAM-safe unlike millis()
    if (n - lastFlowUs > 50000) {            // 50 ms debounce
        pulseCount++;
        lastFlowUs = n;
    }
}

// ─── State ────────────────────────────────────────────────────────────────────
enum ValveState : uint8_t { VALVE_STOPPED, VALVE_OPENING, VALVE_CLOSING, VALVE_OPEN, VALVE_CLOSED };
ValveState valve1State       = VALVE_STOPPED;
uint32_t   valve1MoveStartMs = 0;

esp_timer_handle_t valveStopTimer      = nullptr;
volatile bool      valveStoppedByTimer = false;

uint32_t dongleCycleIntervalMin = 60;
uint32_t dongleOffDurationMin   = 5;

uint32_t lastPublishMs        = 0;
uint32_t lastDongleCycleMs    = 0;
uint32_t lastConnectAttemptMs = 0;
uint32_t connectBackoffMs     = 5000;
static const uint32_t CONNECT_BACKOFF_MAX = 120000;

bool timeSynced = false;

PowerTier powerTier      = TIER_NORMAL;
float     lastBattV      = 0.0f;

uint32_t lastTierCheckMs = 0;

LedMode  ledMode         = LED_DISCONNECTED;
uint32_t ledToggleMs     = 0;
bool     ledOn           = false;

void setLedMode(LedMode m) {
    if (ledMode != m) { ledMode = m; ledToggleMs = 0; ledOn = false; }
}

const char* tierName(PowerTier t) {
    switch (t) {
        case TIER_CONSERVE:      return "conserve";
        case TIER_CRITICAL:      return "critical";
        case TIER_VALVE_RESERVE: return "valve_reserve";
        default:                 return "normal";
    }
}
const char* powerTierStr() { return tierName(powerTier); }

Preferences      prefs;
WiFiClientSecure secureClient;
PubSubClient     mqttClient(secureClient);

// ─── Utility ──────────────────────────────────────────────────────────────────
// Blocking wait using 1-second ticks so the watchdog is fed throughout.
void waitSeconds(uint32_t seconds) {
    for (uint32_t i = 0; i < seconds; i++) delay(1000);
}

// ─── ADC voltage reads ────────────────────────────────────────────────────────
static uint32_t adcAvgMv(int pin) {
    pinMode(pin, INPUT);
    gpio_pullup_dis((gpio_num_t)pin);
    gpio_pulldown_dis((gpio_num_t)pin);
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) { sum += analogReadMilliVolts(pin); delay(2); }
    return sum / 10;
}

float readSolarVoltage() {
    uint32_t mv = adcAvgMv(SOLAR_VOLTAGE_PIN);
    float v = (mv / 1000.0f) * SOLAR_ADC_SCALE;
    if (v < 6.0f) return 0.0f;
    Serial.printf("[ADC] Solar  pin=%lumV  v=%.2fV\n", mv, v);
    return v;
}

float readBattery24V() {
    uint32_t mv = adcAvgMv(BATTERY_24V_PIN);
    float v = (mv / 1000.0f) * BATT_ADC_SCALE;
    if (v < 6.0f) return 0.0f;
    Serial.printf("[ADC] Battery  pin=%lumV  v=%.2fV\n", mv, v);
    return v;
}

// Piecewise-linear LiFePO4 SoC. Returns 0-110; >100 means charger connected.
int batteryLFPPercent(float v) {
    static const float volts[] = {10.5f, 11.8f, 12.2f, 12.5f, 12.8f, 13.0f, 13.2f, 13.4f, 13.8f};
    static const int   pcts[]  = {   0,     3,     8,    15,    25,    50,    80,   100,   110};
    const int n = 9;
    if (v <= volts[0])   return 0;
    if (v >= volts[n-1]) return 110;
    for (int i = 1; i < n; i++) {
        if (v < volts[i]) {
            float frac = (v - volts[i-1]) / (volts[i] - volts[i-1]);
            return (int)(pcts[i-1] + frac * (pcts[i] - pcts[i-1]));
        }
    }
    return 110;
}

int solarVoltageToPercent(float v) {
    // Rough "panel alive" indicator — true charging state is solar_v vs battery_v server-side.
    return (int)constrain((v / SOLAR_MAX_V) * 100.0f, 0.0f, 100.0f);
}

// ─── Power tier evaluation ────────────────────────────────────────────────────
void updatePowerTier() {
    float v = readBattery24V();   // returns 0.0 if below 6V floor (invalid / disconnected)
    lastBattV = v;

    // INVALID / disconnected reading — never shed.
    if (v < 6.0f) {
        if (powerTier != TIER_NORMAL) {
            Serial.println("[Power] battery reading invalid (<6V) — forcing NORMAL, no shedding");
            powerTier = TIER_NORMAL;
        }
        return;
    }

    PowerTier prev   = powerTier;
    PowerTier target;
    const float HYST = 0.2f;

    // Raw target from thresholds (worsening applies immediately, no hysteresis).
    if      (v < BATT_VALVE_RESERVE_V) target = TIER_VALVE_RESERVE;
    else if (v < BATT_CRITICAL_V)      target = TIER_CRITICAL;
    else if (v < BATT_CONSERVE_V)      target = TIER_CONSERVE;
    else                               target = TIER_NORMAL;

    // Hysteresis on RECOVERY only: block move to a lighter tier unless voltage
    // clears that tier's entry threshold by HYST (~0.2V margin required).
    if (target < powerTier) {
        float recov;
        switch (target) {
            case TIER_NORMAL:   recov = BATT_CONSERVE_V      + HYST; break;
            case TIER_CONSERVE: recov = BATT_CRITICAL_V      + HYST; break;
            default:            recov = BATT_VALVE_RESERVE_V + HYST; break;
        }
        if (v < recov) target = powerTier;   // not enough margin yet — stay put
    }

    powerTier = target;
    if (powerTier != prev) {
        Serial.printf("[Power] tier %s -> %s (%.2fV)\n", tierName(prev), powerTierStr(), v);
    }
}

// ─── Valve hw-timer callback — drops relays independent of loop() ────────────
void IRAM_ATTR valveStopCallback(void* arg) {
    gpio_set_level((gpio_num_t)VALVE_1_OPEN,  0);
    gpio_set_level((gpio_num_t)VALVE_1_CLOSE, 0);
    valveStoppedByTimer = true;
}

// ─── Valve control (same safety logic as OURWater.ino) ───────────────────────
const char* valveStateStr() {
    switch (valve1State) {
        case VALVE_OPENING: return "opening";
        case VALVE_CLOSING: return "closing";
        case VALVE_OPEN:    return "open";
        case VALVE_CLOSED:  return "closed";
        default:            return "stopped";
    }
}

// Valve is the priority load — never gated by power tier.
void safeSetValve(const String& action) {
    if (action == "open") {
        if (valve1State == VALVE_OPEN || valve1State == VALVE_OPENING) {
            Serial.println("[Valve] open ignored — already open/opening");
            return;
        }
        digitalWrite(VALVE_1_CLOSE, LOW);
        delay(200);
        digitalWrite(VALVE_1_OPEN, HIGH);
        valve1State       = VALVE_OPENING;
        valve1MoveStartMs = millis();
        esp_timer_stop(valveStopTimer);
        valveStoppedByTimer = false;
        esp_timer_start_once(valveStopTimer, (uint64_t)VALVE_TRAVEL_MS * 1000ULL);
    } else if (action == "close") {
        if (valve1State == VALVE_CLOSED || valve1State == VALVE_CLOSING) {
            Serial.println("[Valve] close ignored — already closed/closing");
            return;
        }
        digitalWrite(VALVE_1_OPEN, LOW);
        delay(200);
        digitalWrite(VALVE_1_CLOSE, HIGH);
        valve1State       = VALVE_CLOSING;
        valve1MoveStartMs = millis();
        esp_timer_stop(valveStopTimer);
        valveStoppedByTimer = false;
        esp_timer_start_once(valveStopTimer, (uint64_t)VALVE_TRAVEL_MS * 1000ULL);
    } else if (action == "stop") {
        esp_timer_stop(valveStopTimer);
        digitalWrite(VALVE_1_OPEN,  LOW);
        digitalWrite(VALVE_1_CLOSE, LOW);
    }
    Serial.printf("[Valve] %s\n", action.c_str());
}

// ─── MQTT publish ─────────────────────────────────────────────────────────────
void publishStatus(const char* status) {
    char payload[80];
    snprintf(payload, sizeof(payload), "{\"status\":\"%s\"}", status);
    mqttClient.publish(BASE_TOPIC "/status", payload, true);   // retained
    Serial.printf("[MQTT] Status → %s\n", status);
}

void publishData() {
    noInterrupts();
    uint32_t pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float solarV  = readSolarVoltage();
    float batt24V = readBattery24V();

    char payload[384];
    snprintf(payload, sizeof(payload),
        "{\"flow_1\":%lu,"
        "\"valve_1\":\"%s\","
        "\"solar_v\":%.2f,\"solar_pct\":%d,"
        "\"battery_24v_v\":%.2f,\"battery_24v_pct\":%d,"
        "\"power_state\":\"%s\","
        "\"firmware\":\"%s\",\"uptime_s\":%lu}",
        (unsigned long)pulses,
        valveStateStr(),
        solarV,  solarVoltageToPercent(solarV),
        batt24V, batteryLFPPercent(batt24V),
        powerTierStr(),
        FIRMWARE_VER,
        (unsigned long)(millis() / 1000)
    );

    if (mqttClient.publish(BASE_TOPIC "/data", payload)) {
        Serial.println("[MQTT] Data published");
    } else {
        Serial.println("[MQTT] Publish failed");
    }
}

// ─── MQTT incoming message handler ───────────────────────────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char msg[128];
    if (length >= sizeof(msg)) length = sizeof(msg) - 1;
    memcpy(msg, payload, length);
    msg[length] = '\0';
    String t = String(topic);
    String p = String(msg);
    Serial.printf("[MQTT] Rcv: %s | %s\n", topic, msg);

    if (t.endsWith("/valve/1/cmd")) {
        // On boot, valve1State=STOPPED so the retained cmd is acted on once — re-asserts intended state after reboot.
        safeSetValve(p);
    } else if (t.endsWith("/config/dongle_cycle")) {
        int val = p.toInt();
        if (val > 0) {
            dongleCycleIntervalMin = (uint32_t)val;
            prefs.putUInt("cycle_min", dongleCycleIntervalMin);
            Serial.printf("[Config] Dongle cycle → %d min\n", val);
        }
    } else if (t.endsWith("/config/dongle_off")) {
        int val = p.toInt();
        if (val > 0) {
            dongleOffDurationMin = (uint32_t)val;
            prefs.putUInt("off_min", dongleOffDurationMin);
            Serial.printf("[Config] Dongle off duration → %d min\n", val);
        }
    }
}

// ─── Supabase: fetch dongle timing on startup ─────────────────────────────────
void fetchDongleSettings() {
    if (WiFi.status() != WL_CONNECTED) return;
    WiFiClientSecure httpsClient;
    httpsClient.setInsecure();   // one-time startup read; key is already in firmware
    HTTPClient http;
    String url = String(SUPABASE_URL)
        + "/rest/v1/meters?serial_no=eq." + SERIAL_NO
        + "&select=dongle_cycle_interval_min,dongle_off_duration_min&limit=1";
    http.begin(httpsClient, url);
    http.setTimeout(10000);
    http.addHeader("apikey",        SUPABASE_KEY);
    http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));
    int code = http.GET();
    if (code == 200) {
        String body = http.getString();
        Serial.println("[Supabase] " + body);
        const char* k1 = "\"dongle_cycle_interval_min\":";
        const char* k2 = "\"dongle_off_duration_min\":";
        int idx = body.indexOf(k1);
        if (idx >= 0) {
            int val = body.substring(idx + strlen(k1)).toInt();
            if (val > 0) {
                dongleCycleIntervalMin = (uint32_t)val;
                prefs.putUInt("cycle_min", dongleCycleIntervalMin);
            }
        }
        idx = body.indexOf(k2);
        if (idx >= 0) {
            int val = body.substring(idx + strlen(k2)).toInt();
            if (val > 0) {
                dongleOffDurationMin = (uint32_t)val;
                prefs.putUInt("off_min", dongleOffDurationMin);
            }
        }
        Serial.printf("[Supabase] cycle=%lu min  off=%lu min\n",
                      dongleCycleIntervalMin, dongleOffDurationMin);
    } else {
        Serial.printf("[Supabase] GET %d — using cached values\n", code);
    }
    http.end();
}

// ─── SNTP time sync (Botswana = UTC+2, no DST) ───────────────────────────────
void syncTime() {
    configTime(2 * 3600, 0, "pool.ntp.org", "time.google.com");
    struct tm timeinfo;
    uint32_t start = millis();
    while (!getLocalTime(&timeinfo) && millis() - start < 10000) delay(200);
    if (getLocalTime(&timeinfo)) {
        timeSynced = true;
        Serial.printf("[NTP] Synced: %04d-%02d-%02d %02d:%02d:%02d (local, UTC+2)\n",
            timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        Serial.println("[NTP] Sync failed — continuing without wall-clock time");
    }
}

bool getLocalTimeNow(struct tm &out) {
    if (!timeSynced) return false;
    return getLocalTime(&out);
}

// ─── WiFi connect ─────────────────────────────────────────────────────────────
bool connectWiFi() {
    Serial.print("[WiFi] Connecting to " WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connected  IP=%s\n", WiFi.localIP().toString().c_str());
        syncTime();
        return true;
    }
    Serial.println("\n[WiFi] Connect failed");
    return false;
}

// ─── Effective publish interval based on power tier ───────────────────────────
uint32_t effectivePublishIntervalMs() {
    if (TEST_MODE) return 60000UL;                //  bench: 1 min regardless of tier
    switch (powerTier) {
        case TIER_CONSERVE:      return  60UL * 60000UL;    //  1 h
        case TIER_CRITICAL:      return   8UL * 3600000UL;  //  8 h
        case TIER_VALVE_RESERVE: return  24UL * 3600000UL;  // 24 h
        default:                 return  30UL * 60000UL;    // 30 min
    }
}

// ─── MQTT connect (includes LWT registration) ────────────────────────────────
bool mqttReconnect() {
    if (WiFi.status() != WL_CONNECTED) {
        if (!connectWiFi()) return false;
        fetchDongleSettings();
    }

    secureClient.setTimeout(10);   // 10 s hard limit on TLS handshake
    Serial.print("[MQTT] Connecting...");
    bool ok = mqttClient.connect(
        MQTT_CLIENT, MQTT_USER, MQTT_PASS,
        BASE_TOPIC "/status",               // LWT topic
        1,                                  // LWT QoS
        true,                               // LWT retain
        "{\"status\":\"offline\"}"          // LWT payload
    );
    if (ok) {
        Serial.println(" OK");
        mqttClient.subscribe(BASE_TOPIC "/#");
        publishStatus("online");
        digitalWrite(STATUS_LED_PIN, HIGH);
        setLedMode(LED_CONNECTED);
        // Only publish on first connect or if the full interval has already elapsed.
        // Reconnects mid-interval (e.g. after a dongle cycle) skip the publish so
        // the flow table only gets records on the regular schedule.
        if (lastPublishMs == 0 || millis() - lastPublishMs >= effectivePublishIntervalMs()) {
            publishData();
            lastPublishMs = millis();
        }
        return true;
    }
    Serial.printf(" FAILED rc=%d\n", mqttClient.state());
    return false;
}

// ─── Dongle power cycle ───────────────────────────────────────────────────────
// Simulates pressing the dongle's power button via an NPN transistor:
//   BASE HIGH (3 s) → transistor ON → button pulled to GND → dongle registers press
//   BASE LOW        → transistor OFF → button released
void doDongleCycle() {
    Serial.println("[Dongle] Starting cycle");
    esp_task_wdt_delete(NULL);   // detach for the full cycle (~7.5 min) — re-added at exit

    if (mqttClient.connected()) {
        publishStatus("dongle_cycling");
        mqttClient.disconnect();
    }
    digitalWrite(STATUS_LED_PIN, LOW);
    setLedMode(LED_DONGLING);

    // Step 1 — press button to power off dongle
    Serial.println("[Dongle] Button press — power OFF");
    digitalWrite(DONGLE_BUTTON_PIN, HIGH);
    delay(3000);
    digitalWrite(DONGLE_BUTTON_PIN, LOW);

    // Step 2 — wait for dongle to fully shut down
    Serial.printf("[Dongle] Off for %lu min...\n", dongleOffDurationMin);
    waitSeconds(dongleOffDurationMin * 60);

    // Step 3 — press button to power dongle back on
    Serial.println("[Dongle] Button press — power ON");
    digitalWrite(DONGLE_BUTTON_PIN, HIGH);
    delay(3000);
    digitalWrite(DONGLE_BUTTON_PIN, LOW);

    // Step 4 — wait for dongle to boot and register with network
    Serial.println("[Dongle] Waiting 60 s for dongle boot...");
    waitSeconds(60);

    // Step 5 — reconnect WiFi
    WiFi.disconnect();
    delay(1000);
    bool wifiOk = connectWiFi();

    // Step 6 — reconnect MQTT (WiFi already attempted above; skip re-check in mqttReconnect)
    if (wifiOk) {
        secureClient.setTimeout(10);
        if (!mqttClient.connect(
                MQTT_CLIENT, MQTT_USER, MQTT_PASS,
                BASE_TOPIC "/status", 1, true, "{\"status\":\"offline\"}")) {
            Serial.printf("[Dongle] MQTT connect failed rc=%d — will retry in loop\n", mqttClient.state());
        } else {
            Serial.println("[Dongle] MQTT reconnected OK");
            mqttClient.subscribe(BASE_TOPIC "/#");
            publishStatus("online");
            setLedMode(LED_CONNECTED);
        }
    } else {
        Serial.println("[Dongle] WiFi failed — MQTT will retry in loop");
    }
    esp_task_wdt_add(NULL);    // re-subscribe loop task before returning to loop()
    esp_task_wdt_reset();
}

// ─── Valve service — reconciles state after hw-timer fires, sw-fallback if not ──
void serviceValve() {
    if (valveStoppedByTimer) {
        valveStoppedByTimer = false;
        if (valve1State == VALVE_OPENING)      valve1State = VALVE_OPEN;
        else if (valve1State == VALVE_CLOSING) valve1State = VALVE_CLOSED;
        Serial.printf("[Valve] travel complete (hw-timer) -> %s\n", valveStateStr());
        return;
    }
    if (valve1State != VALVE_OPENING && valve1State != VALVE_CLOSING) return;
    if (millis() - valve1MoveStartMs < VALVE_TRAVEL_MS) return;
    bool wasOpening = (valve1State == VALVE_OPENING);
    safeSetValve("stop");
    valve1State = wasOpening ? VALVE_OPEN : VALVE_CLOSED;
    Serial.printf("[Valve] travel complete (sw-fallback) -> %s\n", valveStateStr());
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=============================");
    Serial.println(" OURWater Super Mini v" FIRMWARE_VER);
    Serial.println("=============================");

    // Validate RTC-retained pulse count. On a cold/power-on boot the RTC RAM
    // contains garbage, so guard with a magic marker before trusting the value.
    if (pulseCountMagic != PULSE_MAGIC) {
        pulseCount      = 0;
        pulseCountMagic = PULSE_MAGIC;
        Serial.println("[Flow] Cold boot — pulse count reset to 0");
    } else {
        Serial.printf("[Flow] Warm boot — retained pulse count = %lu\n", (unsigned long)pulseCount);
    }

    // Flow sensor — use ESP-IDF GPIO driver directly; Arduino attachInterrupt()
    // silently fails on this Super Mini variant.
    const gpio_config_t flow_conf = {
        .pin_bit_mask  = (1ULL << FLOW_1_PIN),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&flow_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)FLOW_1_PIN, onFlow1, NULL);
    Serial.println("[GPIO] Flow sensor ready (GPIO10, ESP-IDF ISR)");

    // Output pins — all LOW on boot
    neopixelWrite(NEO_PIN, 0, 0, 0);   // clear WS2812B before it latches noise
    pinMode(VALVE_1_OPEN,      OUTPUT); digitalWrite(VALVE_1_OPEN,      LOW);
    pinMode(VALVE_1_CLOSE,     OUTPUT); digitalWrite(VALVE_1_CLOSE,     LOW);
    pinMode(DONGLE_BUTTON_PIN, OUTPUT); digitalWrite(DONGLE_BUTTON_PIN, LOW);
    pinMode(STATUS_LED_PIN,    OUTPUT); digitalWrite(STATUS_LED_PIN,    LOW);
    Serial.println("[GPIO] Outputs ready");

    // One-shot hw-timer: drops valve relays at travel deadline even while loop() is blocked.
    {
        const esp_timer_create_args_t valveTimerArgs = {
            .callback        = &valveStopCallback,
            .arg             = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name            = "valve_stop",
        };
        esp_timer_create(&valveTimerArgs, &valveStopTimer);
        Serial.println("[Valve] hw-timer created");
    }

    // ADC calibration — set BATT_ADC_SCALE / SOLAR_ADC_SCALE to match real divider.
    // Procedure: measure true voltage with a multimeter, read pin_mV from serial,
    // then: true_scale = V_multimeter / (pin_mV / 1000.0)
    {
        uint32_t battMv  = adcAvgMv(BATTERY_24V_PIN);
        uint32_t solarMv = adcAvgMv(SOLAR_VOLTAGE_PIN);
        Serial.printf("[CAL] Battery: pin=%lumV  scale=%.1f  =>  reported=%.2fV\n",
                      battMv,  BATT_ADC_SCALE,  (battMv  / 1000.0f) * BATT_ADC_SCALE);
        Serial.printf("[CAL] Solar:   pin=%lumV  scale=%.1f  =>  reported=%.2fV\n",
                      solarMv, SOLAR_ADC_SCALE, (solarMv / 1000.0f) * SOLAR_ADC_SCALE);
        Serial.println("[CAL] To calibrate: true_scale = V_multimeter / (pin_mV / 1000)");
    }

    // Load persisted dongle settings (fallback if Supabase unreachable)
    prefs.begin("ourwater_sm", false);
    dongleCycleIntervalMin = prefs.getUInt("cycle_min", 60);
    dongleOffDurationMin   = prefs.getUInt("off_min",   5);
    Serial.printf("[Config] Dongle cycle=%lu min  off=%lu min  TEST_MODE=%s\n",
                  dongleCycleIntervalMin, dongleOffDurationMin,
                  TEST_MODE ? "ON" : "OFF");

    // Configure MQTT client
    // EMQX Serverless uses its own CA — skip cert verification (connection is still encrypted).
    secureClient.setInsecure();
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);
    mqttClient.setKeepAlive(60);

    // Arm dongle cycle timer from now so first cycle fires after full interval
    lastDongleCycleMs = millis();

    // Take ownership of the Task Watchdog (core 3.x / IDF5 config-struct API).
    // ESP_ERR_INVALID_STATE means the core already initialised the TWDT — reconfigure it.
    {
        esp_task_wdt_config_t wdtCfg = {
            .timeout_ms     = WDT_TIMEOUT_S * 1000,
            .idle_core_mask = 0,      // do not watch idle tasks, only our subscribed task
            .trigger_panic  = true,   // panic + reset on timeout
        };
        esp_err_t rc = esp_task_wdt_init(&wdtCfg);
        if (rc == ESP_ERR_INVALID_STATE) {
            esp_task_wdt_reconfigure(&wdtCfg);
        }
        esp_task_wdt_add(NULL);    // subscribe the loop task
        esp_task_wdt_reset();
        Serial.printf("[WDT] Task watchdog armed: %d s, loop task subscribed\n", WDT_TIMEOUT_S);
    }

    Serial.println("[Boot] System ready");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    esp_task_wdt_reset();   // feed the watchdog every iteration
    serviceValve();   // first — de-energises relay on travel timeout even while MQTT is down
    serviceLED();     // non-blocking RGB status update

    // Connection manager — non-blocking backoff between attempts.
    // serviceValve/serviceLED above run every iteration regardless of connection state.
    if (!mqttClient.connected()) {
        digitalWrite(STATUS_LED_PIN, LOW);
        setLedMode(LED_DISCONNECTED);
        if (millis() - lastConnectAttemptMs >= connectBackoffMs) {
            lastConnectAttemptMs = millis();
            esp_task_wdt_delete(NULL);        // detach during blocking connect (WiFi + TLS)
            bool ok = mqttReconnect();
            esp_task_wdt_add(NULL);           // re-subscribe immediately after, success or fail
            esp_task_wdt_reset();
            if (ok) {
                connectBackoffMs = 5000;
            } else {
                connectBackoffMs *= 2;
                if (connectBackoffMs > CONNECT_BACKOFF_MAX) connectBackoffMs = CONNECT_BACKOFF_MAX;
                Serial.printf("[Net] Next retry in %lu s\n", connectBackoffMs / 1000);
            }
        }
        return;
    }

    mqttClient.loop();   // process incoming messages and keep-alive

    // Power tier check — every 60 s (ADC reads have settling delays; not every iteration)
    if (millis() - lastTierCheckMs >= 60000UL) {
        lastTierCheckMs = millis();
        updatePowerTier();
    }

    // Periodic data publish — interval driven by power tier
    if (millis() - lastPublishMs >= effectivePublishIntervalMs()) {
        publishData();
        lastPublishMs = millis();
    }

    // Dongle power cycle — suppressed in critical/valve_reserve to save power
    uint32_t cycleIntervalMs = TEST_MODE
        ? 5UL * 60000UL
        : dongleCycleIntervalMin * 60000UL;
    bool dongleCycleAllowed = (powerTier != TIER_CRITICAL && powerTier != TIER_VALVE_RESERVE);
    if (dongleCycleAllowed && dongleCycleIntervalMin > 0 && millis() - lastDongleCycleMs >= cycleIntervalMs) {
        doDongleCycle();
        lastDongleCycleMs = millis();
    }

    delay(100);
}
