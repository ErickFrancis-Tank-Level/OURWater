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
#include <Wire.h>
#include <time.h>
#include "driver/gpio.h"
#include "board_config.h"

#define FIRMWARE_VER    "SM-1.0.0"
#define VALVE_TRAVEL_MS  15000    // ~15 s end-to-end travel; relay de-energised by serviceValve()

// ─── Pin assignments ──────────────────────────────────────────────────────────
#define FLOW_1_PIN         10
#define VALVE_1_OPEN        8
#define VALVE_1_CLOSE      12
#define SOLAR_VOLTAGE_PIN   3
#define BATTERY_24V_PIN     4
#define DONGLE_BUTTON_PIN   5
#define STATUS_LED_PIN     13
#define BATTERY_SDA         6
#define BATTERY_SCL         7

// ─── ADC calibration (same formulas as OURWater.ino) ─────────────────────────
#define SOLAR_ADC_SCALE   28.1f
#define BATT24V_ADC_SCALE 11.0f
#define BATT24V_FULL_V    25.6f
#define BATT24V_EMPTY_V   21.0f
#define SOLAR_MAX_V       40.0f

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
volatile uint32_t pulseCount  = 0;
volatile uint32_t lastFlowMs  = 0;

void IRAM_ATTR onFlow1(void* arg) {
    uint32_t n = millis();
    if (n - lastFlowMs > 50) {
        pulseCount++;
        lastFlowMs = n;
    }
}

// ─── State ────────────────────────────────────────────────────────────────────
enum ValveState : uint8_t { VALVE_STOPPED, VALVE_OPENING, VALVE_CLOSING, VALVE_OPEN, VALVE_CLOSED };
ValveState valve1State     = VALVE_STOPPED;
uint32_t   valve1MoveStartMs = 0;

uint8_t max17048Addr = 0x36;
float   battPct      = 0.0f;
float   battVoltage  = 0.0f;
bool    battGaugeOk  = false;

uint32_t dongleCycleIntervalMin = 60;
uint32_t dongleOffDurationMin   = 5;

uint32_t lastPublishMs     = 0;
uint32_t lastDongleCycleMs = 0;

bool timeSynced = false;

Preferences      prefs;
WiFiClientSecure secureClient;
PubSubClient     mqttClient(secureClient);

// ─── Utility ──────────────────────────────────────────────────────────────────
// Blocking wait using 1-second ticks so the watchdog is fed throughout.
void waitSeconds(uint32_t seconds) {
    for (uint32_t i = 0; i < seconds; i++) delay(1000);
}

// ─── MAX17048 battery gauge (same logic as OURWater.ino) ─────────────────────
bool probeMAX17048() {
    Wire.beginTransmission(0x36);
    if (Wire.endTransmission() == 0) { max17048Addr = 0x36; return true; }
    return false;
}

bool readMAX17048(float &pct, float &voltage) {
    Wire.beginTransmission(max17048Addr);
    Wire.write(0x02);                               // VCELL register
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(max17048Addr, (uint8_t)2);
    if (Wire.available() < 2) return false;
    uint16_t rawV = ((uint16_t)Wire.read() << 8) | Wire.read();
    voltage = rawV * 1.25f / 1000.0f;              // 1.25 mV per LSB

    Wire.beginTransmission(max17048Addr);
    Wire.write(0x04);                               // SOC register
    Wire.endTransmission(false);
    Wire.requestFrom(max17048Addr, (uint8_t)2);
    if (Wire.available() < 2) return false;
    pct = (float)Wire.read() + (float)Wire.read() / 256.0f;
    return true;
}

void updateBattery() {
    if (!battGaugeOk && !probeMAX17048()) return;
    float p = battPct, v = battVoltage;
    if (!readMAX17048(p, v)) { battGaugeOk = false; return; }
    battGaugeOk = true;
    battPct     = p;
    battVoltage = v;
    Serial.printf("[MAX17048] %.1f%%  %.3fV\n", battPct, battVoltage);
}

// ─── ADC voltage reads (same formulas as OURWater.ino) ───────────────────────
float readSolarVoltage() {
    pinMode(SOLAR_VOLTAGE_PIN, INPUT);
    gpio_pullup_dis((gpio_num_t)SOLAR_VOLTAGE_PIN);
    gpio_pulldown_dis((gpio_num_t)SOLAR_VOLTAGE_PIN);
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) { sum += analogRead(SOLAR_VOLTAGE_PIN); delay(2); }
    int raw = (int)(sum / 10);
    if (raw < 100) return 0.0f;
    float v = raw * 3.3f / 4095.0f * SOLAR_ADC_SCALE;
    if (v < 10.0f) return 0.0f;
    Serial.printf("[ADC] Solar  raw=%d  v=%.2fV\n", raw, v);
    return v;
}

float readBattery24V() {
    pinMode(BATTERY_24V_PIN, INPUT);
    gpio_pullup_dis((gpio_num_t)BATTERY_24V_PIN);
    gpio_pulldown_dis((gpio_num_t)BATTERY_24V_PIN);
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) { sum += analogRead(BATTERY_24V_PIN); delay(2); }
    int raw = (int)(sum / 10);
    if (raw < 100) return 0.0f;
    float v = raw * 3.3f / 4095.0f * BATT24V_ADC_SCALE;
    if (v < 10.0f) return 0.0f;
    Serial.printf("[ADC] Batt24V raw=%d  v=%.2fV\n", raw, v);
    return v;
}

int voltage24VToPercent(float v) {
    float pct = (v - BATT24V_EMPTY_V) / (BATT24V_FULL_V - BATT24V_EMPTY_V) * 100.0f;
    return (int)constrain(pct, 0.0f, 100.0f);
}

int solarVoltageToPercent(float v) {
    return (int)constrain((v / SOLAR_MAX_V) * 100.0f, 0.0f, 100.0f);
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

void safeSetValve(const String& action) {
    if (action == "open") {
        digitalWrite(VALVE_1_CLOSE, LOW);
        delay(200);
        digitalWrite(VALVE_1_OPEN, HIGH);
        valve1State      = VALVE_OPENING;
        valve1MoveStartMs = millis();
    } else if (action == "close") {
        digitalWrite(VALVE_1_OPEN, LOW);
        delay(200);
        digitalWrite(VALVE_1_CLOSE, HIGH);
        valve1State      = VALVE_CLOSING;
        valve1MoveStartMs = millis();
    } else if (action == "stop") {
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

    updateBattery();
    float solarV  = readSolarVoltage();
    float batt24V = readBattery24V();

    char bPctStr[8], bVStr[12];
    if (battGaugeOk) {
        snprintf(bPctStr, sizeof(bPctStr), "%d",   (int)battPct);
        snprintf(bVStr,   sizeof(bVStr),   "%.2f", battVoltage);
    } else {
        strcpy(bPctStr, "null");
        strcpy(bVStr,   "null");
    }

    char payload[384];
    snprintf(payload, sizeof(payload),
        "{\"flow_1\":%lu,"
        "\"valve_1\":\"%s\","
        "\"battery_pct\":%s,\"battery_v\":%s,"
        "\"solar_v\":%.2f,\"solar_pct\":%d,"
        "\"battery_24v_v\":%.2f,\"battery_24v_pct\":%d,"
        "\"firmware\":\"%s\",\"uptime_s\":%lu}",
        (unsigned long)pulses,
        valveStateStr(),
        bPctStr, bVStr,
        solarV,  solarVoltageToPercent(solarV),
        batt24V, voltage24VToPercent(batt24V),
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

// ─── MQTT connect (includes LWT registration) ────────────────────────────────
bool mqttReconnect() {
    if (WiFi.status() != WL_CONNECTED) {
        if (!connectWiFi()) return false;
        fetchDongleSettings();
    }

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
        // Only publish on first connect or if the full interval has already elapsed.
        // Reconnects mid-interval (e.g. after a dongle cycle) skip the publish so
        // the flow table only gets records on the regular schedule.
        uint32_t intervalMs = TEST_MODE ? 60000UL : 30UL * 60000UL;
        if (lastPublishMs == 0 || millis() - lastPublishMs >= intervalMs) {
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

    if (mqttClient.connected()) {
        publishStatus("dongle_cycling");
        mqttClient.disconnect();
    }
    digitalWrite(STATUS_LED_PIN, LOW);

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
    connectWiFi();

    // Step 6 — reconnect MQTT
    if (!mqttReconnect()) {
        Serial.println("[Dongle] MQTT reconnect failed — will retry in loop");
    }
}

// ─── Non-blocking valve travel auto-stop ─────────────────────────────────────
void serviceValve() {
    if (valve1State != VALVE_OPENING && valve1State != VALVE_CLOSING) return;
    if (millis() - valve1MoveStartMs < VALVE_TRAVEL_MS) return;
    bool wasOpening = (valve1State == VALVE_OPENING);
    safeSetValve("stop");
    valve1State = wasOpening ? VALVE_OPEN : VALVE_CLOSED;
    Serial.printf("[Valve] travel complete -> %s (relay off)\n",
                  wasOpening ? "open" : "closed");
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=============================");
    Serial.println(" OURWater Super Mini v" FIRMWARE_VER);
    Serial.println("=============================");

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
    pinMode(VALVE_1_OPEN,      OUTPUT); digitalWrite(VALVE_1_OPEN,      LOW);
    pinMode(VALVE_1_CLOSE,     OUTPUT); digitalWrite(VALVE_1_CLOSE,     LOW);
    pinMode(DONGLE_BUTTON_PIN, OUTPUT); digitalWrite(DONGLE_BUTTON_PIN, LOW);
    pinMode(STATUS_LED_PIN,    OUTPUT); digitalWrite(STATUS_LED_PIN,    LOW);
    Serial.println("[GPIO] Outputs ready");

    // I2C — MAX17048 battery gauge
    gpio_set_pull_mode((gpio_num_t)BATTERY_SDA, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)BATTERY_SCL, GPIO_PULLUP_ONLY);
    Wire.begin(BATTERY_SDA, BATTERY_SCL);
    Wire.setClock(100000);
    delay(100);
    if (probeMAX17048()) {
        Serial.println("[I2C] MAX17048 found at 0x36");
    } else {
        Serial.println("[I2C] MAX17048 not found — will retry in loop");
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

    // Connect WiFi, fetch fresh settings from Supabase, connect MQTT
    if (connectWiFi()) {
        fetchDongleSettings();
    }
    mqttReconnect();

    // Arm dongle cycle timer from now so first cycle fires after full interval
    lastDongleCycleMs = millis();

    Serial.println("[Boot] System ready");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    serviceValve();   // first — de-energises relay on travel timeout even while MQTT is down

    // Reconnect if disconnected
    if (!mqttClient.connected()) {
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(5000);
        mqttReconnect();
        return;
    }

    mqttClient.loop();   // process incoming messages and keep-alive

    // Periodic data publish
    uint32_t publishIntervalMs = TEST_MODE ? 60000UL : 30UL * 60000UL;
    if (millis() - lastPublishMs >= publishIntervalMs) {
        publishData();
        lastPublishMs = millis();
    }

    // Dongle power cycle
    uint32_t cycleIntervalMs = TEST_MODE
        ? 5UL * 60000UL
        : dongleCycleIntervalMin * 60000UL;
    if (dongleCycleIntervalMin > 0 && millis() - lastDongleCycleMs >= cycleIntervalMs) {
        doDongleCycle();
        lastDongleCycleMs = millis();   // reset after cycle completes
    }

    delay(100);
}
