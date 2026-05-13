#include <Preferences.h>
#include <Wire.h>
#include "driver/gpio.h"
#include "esp_sleep.h"

HardwareSerial modemSerial(1);
Preferences    prefs;

// ─── Pin Definitions ──────────────────────────────────────────────────────────
#define MODEM_POWER      33
#define MODEM_RX         17
#define MODEM_TX         18
#define FLOW_1           21
#define FLOW_2           40
#define FLOW_3           41
#define FLOW_4           39
#define VALVE_1_OPEN     45
#define VALVE_1_CLOSE    35
#define VALVE_2_OPEN     36
#define VALVE_2_CLOSE    37
#define PRESSURE_PIN      1
#define SONAR_TRIG       42
#define SONAR_ECHO       34
#define BATTERY_SDA      15
#define BATTERY_SCL      16
#define SOLAR_VOLTAGE_PIN  8   // IO8 — solar panel voltage divider
#define BATTERY_24V_PIN    9   // IO9 — 24V battery voltage divider
#define TEST_MODE        true
#define CUBIC_METRES_PER_PULSE 0.001f   // 1 pulse = 1 L; conversion to m³ happens in dashboard

// ─── Constants ────────────────────────────────────────────────────────────────
#define FIRMWARE_VER  "1.2.0"

#define MQTT_BROKER   "tcp://g7214f51.ala.eu-central-1.emqxsl.com:8883"
#define MQTT_USER     "ourwater_esp32"
#define MQTT_PASS     "@1Mandela1234"
#define MQTT_CLIENT   "ourwater_001"
#define BASE_TOPIC    "ourwater/ourwater_botswana/site_001_meter_01"

#define BATT_LOW_PCT     70
#define VALVE_PULSE_MS  500
#define PRESSURE_MAX_BAR 10.0f   // full-scale of the 4-20mA transducer

const uint8_t VALID_INTERVALS[] = {1, 15, 30, 60, 120};
#define NUM_VALID_INTERVALS 5

// ─── Flow counters ────────────────────────────────────────────────────────────
volatile uint32_t pulseCount[4]  = {0, 0, 0, 0};
volatile uint32_t lastFlowMs[4]  = {0, 0, 0, 0};
uint32_t          totalPulses[4] = {0, 0, 0, 0};

// ─── Device state ─────────────────────────────────────────────────────────────
enum ValveState  : uint8_t { VALVE_STOPPED, VALVE_OPENING, VALVE_CLOSING };
enum PowerState  : uint8_t { PWR_NORMAL, PWR_WARNING, PWR_CRITICAL, PWR_INTERNAL_ONLY };

ValveState valveState[2]     = {VALVE_STOPPED, VALVE_STOPPED};
PowerState powerState        = PWR_NORMAL;
int        last24VPct        = 100;

uint8_t    max17048Addr      = 0x36;
float    battPct             = 100.0f;
float    battVoltage         = 0.0f;
bool     onBackup            = false;
uint8_t  configuredInterval  = 30;
bool     mqttConnected       = false;
uint8_t  mqttFailCount       = 0;
uint32_t lastBattReadMs      = 0;
uint32_t lastAdcPrintMs      = 0;
uint32_t lastAlarmMs         = 0;

// NTP / publish scheduling
uint32_t ntpSecondsOfDay = 0;
uint32_t ntpBootMs       = 0;
uint32_t nextPublishMs   = 0;

// ─── Alarm state ──────────────────────────────────────────────────────────────
float    prevSolarV       = -1.0f;   // negative = not yet sampled
float    prevBatt24V      = -1.0f;
uint32_t prevSolarAboveMs = 0;       // millis() when solar was last > 5V
bool     solarAlarm       = false;
bool     battAlarm        = false;
float    current24V       = 0.0f;    // last measured 24V battery voltage

// ─── CA Certificate ───────────────────────────────────────────────────────────
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

// ─── Flow ISRs ────────────────────────────────────────────────────────────────
void IRAM_ATTR onFlow1() { uint32_t n=millis(); if(n-lastFlowMs[0]>50){pulseCount[0]++;lastFlowMs[0]=n;} }
void IRAM_ATTR onFlow2() { uint32_t n=millis(); if(n-lastFlowMs[1]>50){pulseCount[1]++;lastFlowMs[1]=n;} }
void IRAM_ATTR onFlow3() { uint32_t n=millis(); if(n-lastFlowMs[2]>50){pulseCount[2]++;lastFlowMs[2]=n;} }
void IRAM_ATTR onFlow4() { uint32_t n=millis(); if(n-lastFlowMs[3]>50){pulseCount[3]++;lastFlowMs[3]=n;} }

// ─── AT helpers ───────────────────────────────────────────────────────────────
String sendAT(const char* cmd, const char* expected, uint32_t timeout) {
    while (modemSerial.available()) modemSerial.read();
    modemSerial.println(cmd);
    Serial.print(">> "); Serial.println(cmd);
    String response = "";
    uint32_t start = millis();
    while (millis() - start < timeout) {
        while (modemSerial.available()) response += (char)modemSerial.read();
        if (response.indexOf(expected) >= 0) break;
        if (response.indexOf("ERROR") >= 0) break;
        yield();
    }
    Serial.print("<< "); Serial.println(response);
    return response;
}

void sendATData(const char* cmd, const char* data, const char* expected, uint32_t timeout) {
    while (modemSerial.available()) modemSerial.read();
    modemSerial.println(cmd);
    Serial.print(">> "); Serial.println(cmd);
    delay(500);
    modemSerial.println(data);
    Serial.print(">> [data] "); Serial.println(data);
    String response = "";
    uint32_t start = millis();
    while (millis() - start < timeout) {
        while (modemSerial.available()) response += (char)modemSerial.read();
        if (response.indexOf(expected) >= 0) break;
        if (response.indexOf("ERROR") >= 0) break;
        yield();
    }
    Serial.print("<< "); Serial.println(response);
}

// ─── Sensors ──────────────────────────────────────────────────────────────────
float readPressureBar() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += analogRead(PRESSURE_PIN);
    float voltage   = (float)(sum / 16) / 4095.0f * 3.3f;
    float currentMA = voltage / 165.0f * 1000.0f;   // 165Ω shunt: I = V/R
    float bar       = (currentMA - 4.0f) / 16.0f * PRESSURE_MAX_BAR;
    return constrain(bar, 0.0f, PRESSURE_MAX_BAR);
}

float readSonarCm() {
    digitalWrite(SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG, LOW);
    long duration = pulseIn(SONAR_ECHO, HIGH, 30000);   // 30 ms timeout
    if (duration == 0) return -1.0f;
    return duration * 0.01715f;   // µs × (0.0343 cm/µs ÷ 2)
}

void probeMAX17048() {
    const uint8_t candidates[2] = {0x36, 0x6C};
    bool found = false;
    for (int i = 0; i < 2; i++) {
        Wire.beginTransmission(candidates[i]);
        byte err = Wire.endTransmission();
        Serial.printf("[I2C] Probe 0x%02X: %s\n", candidates[i], err == 0 ? "ACK ✓" : "NACK");
        if (err == 0 && !found) {
            max17048Addr = candidates[i];
            found = true;
        }
    }
    if (found) {
        Serial.printf("[I2C] MAX17048 using address 0x%02X\n", max17048Addr);
    } else {
        Serial.println("[I2C] MAX17048 not found — check wiring on SDA=15 SCL=16");
    }
}

bool readMAX17048(float &pct, float &voltage) {
    Wire.beginTransmission(max17048Addr);
    Wire.write(0x02);                             // VCELL register
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(max17048Addr, (uint8_t)2);
    if (Wire.available() < 2) return false;
    uint16_t rawV = ((uint16_t)Wire.read() << 8) | Wire.read();
    voltage = rawV * 1.25f / 1000.0f;            // 1.25 mV per LSB
    Serial.printf("[MAX17048] VCELL raw=0x%04X voltage=%.3fV\n", rawV, voltage);

    Wire.beginTransmission(max17048Addr);
    Wire.write(0x04);                             // SOC register
    Wire.endTransmission(false);
    Wire.requestFrom(max17048Addr, (uint8_t)2);
    if (Wire.available() < 2) return false;
    pct = (float)Wire.read() + (float)Wire.read() / 256.0f;
    Serial.printf("[MAX17048] SOC=%.1f%%\n", pct);
    return true;
}

void updateBattery() {
    float newPct = battPct, newVoltage = battVoltage;
    if (!readMAX17048(newPct, newVoltage)) {
        Serial.println("[Batt] MAX17048 read failed");
        return;
    }
    onBackup    = (newPct < battPct - 0.2f);     // actively discharging → backup
    battPct     = newPct;
    battVoltage = newVoltage;
}

// ─── Solar / 24V battery voltage ─────────────────────────────────────────────
// Solar (IO8): divider 220k+51k top, 10k bottom → scale = 281/10 = 28.1
// 24V bat (IO9): divider 100k top, 10k bottom → scale = 110/10 = 11.0
float readSolarVoltage() {
    pinMode(SOLAR_VOLTAGE_PIN, INPUT);
    gpio_pullup_dis(GPIO_NUM_8);
    gpio_pulldown_dis(GPIO_NUM_8);
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) { sum += analogRead(SOLAR_VOLTAGE_PIN); delay(2); }
    int raw = (int)(sum / 10);
    if (raw < 100) {
        Serial.printf("[ADC] Solar    IO8  raw=%4d  (floating — reporting 0.0V)\n", raw);
        return 0.0f;
    }
    float v_pin   = raw * 3.3f / 4095.0f;
    float v_solar = v_pin * (281.0f / 10.0f);
    Serial.printf("[ADC] Solar    IO8  raw=%4d  v_pin=%.3fV  v_solar=%.2fV\n", raw, v_pin, v_solar);
    return v_solar;
}

float readBattery24V() {
    pinMode(BATTERY_24V_PIN, INPUT);
    gpio_pullup_dis(GPIO_NUM_9);
    gpio_pulldown_dis(GPIO_NUM_9);
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) { sum += analogRead(BATTERY_24V_PIN); delay(2); }
    int raw = (int)(sum / 10);
    if (raw < 100) {
        Serial.printf("[ADC] Batt24V  IO9  raw=%4d  (floating — reporting 0.0V)\n", raw);
        return 0.0f;
    }
    float v_pin    = raw * 3.3f / 4095.0f;
    float v_batt24 = v_pin * (110.0f / 10.0f);
    if (v_batt24 < 10.0f) {
        Serial.printf("[ADC] Batt24V  IO9  raw=%4d  v_batt=%.2fV  (below valid range — reporting 0.0V)\n", raw, v_batt24);
        return 0.0f;
    }
    Serial.printf("[ADC] Batt24V  IO9  raw=%4d  v_pin=%.3fV  v_batt=%.2fV\n", raw, v_pin, v_batt24);
    return v_batt24;
}

int voltage24VToPercent(float v) {
    // 24V lead-acid: 25.6V = 100%, 21.0V = 0%
    float pct = (v - 21.0f) / (25.6f - 21.0f) * 100.0f;
    if (pct > 100.0f) pct = 100.0f;
    if (pct <   0.0f) pct =   0.0f;
    return (int)pct;
}

int solarVoltageToPercent(float v) {
    // 2× panels in series: ~40V peak
    float pct = (v / 40.0f) * 100.0f;
    if (pct > 100.0f) pct = 100.0f;
    if (pct <   0.0f) pct =   0.0f;
    return (int)pct;
}

// ─── Cable cut alarm detection ────────────────────────────────────────────────
void checkAlarmConditions(float solarV, float batt24V) {
    uint32_t now = millis();

    // Solar: track last time above threshold; alarm if drops below 0.5V within 3s
    if (solarV > 5.0f) {
        prevSolarAboveMs = now;
        if (solarAlarm) {
            solarAlarm = false;
            Serial.println("[Alarm] Solar voltage recovered");
            if (mqttConnected) publishStatus("solar_alarm_cleared");
        }
    } else if (solarV < 0.5f && prevSolarAboveMs > 0 && (now - prevSolarAboveMs) < 3000) {
        if (!solarAlarm) {
            solarAlarm = true;
            Serial.println("[Alarm] Solar cable cut detected!");
            if (mqttConnected) publishStatus("solar_cable_cut");
        }
    }

    // Battery 24V: alarm if drops from > 20V to < 1V
    if (batt24V > 20.0f) {
        if (battAlarm) {
            battAlarm = false;
            Serial.println("[Alarm] Battery 24V voltage recovered");
            if (mqttConnected) publishStatus("battery_alarm_cleared");
        }
    } else if (prevBatt24V > 20.0f && batt24V < 1.0f) {
        if (!battAlarm) {
            battAlarm = true;
            Serial.println("[Alarm] Battery 24V cable cut detected!");
            if (mqttConnected) publishStatus("battery_cable_cut");
        }
    }

    prevSolarV  = solarV;
    prevBatt24V = batt24V;
}

void checkAlarms() {
    float solarV  = readSolarVoltage();
    float batt24V = readBattery24V();
    current24V    = batt24V;
    last24VPct    = voltage24VToPercent(batt24V);
    checkAlarmConditions(solarV, batt24V);
}

// ─── Power state helpers ──────────────────────────────────────────────────────
const char* powerStateStr() {
    switch (powerState) {
        case PWR_WARNING:       return "warning";
        case PWR_CRITICAL:      return "critical";
        case PWR_INTERNAL_ONLY: return "internal_only";
        default:                return "normal";
    }
}

// ─── Interval helpers ─────────────────────────────────────────────────────────
bool isValidInterval(uint8_t v) {
    for (int i = 0; i < NUM_VALID_INTERVALS; i++)
        if (VALID_INTERVALS[i] == v) return true;
    return false;
}

uint8_t getActiveInterval() {
    if (TEST_MODE) return 1;
    if (powerState == PWR_WARNING || powerState == PWR_INTERNAL_ONLY)
        return configuredInterval > 60 ? configuredInterval : 60;
    if (onBackup || battPct < (float)BATT_LOW_PCT)
        return configuredInterval > 60 ? configuredInterval : 60;
    return configuredInterval;
}

// ─── NTP / scheduling ─────────────────────────────────────────────────────────
uint32_t parseClockToSecondsOfDay(const String& resp) {
    // +CCLK: "26/05/04,22:36:22+00"
    int comma = resp.indexOf(',');
    if (comma < 0) return 0;
    int h = resp.substring(comma + 1, comma + 3).toInt();
    int m = resp.substring(comma + 4, comma + 6).toInt();
    int s = resp.substring(comma + 7, comma + 9).toInt();
    return (uint32_t)h * 3600 + (uint32_t)m * 60 + (uint32_t)s;
}

uint32_t getCurrentSecondsOfDay() {
    if (ntpBootMs == 0) return 0;
    uint32_t elapsed = (millis() - ntpBootMs) / 1000;
    return (ntpSecondsOfDay + elapsed) % 86400;
}

void scheduleNextPublish() {
    uint8_t  interval     = getActiveInterval();
    uint32_t sod          = getCurrentSecondsOfDay();
    uint32_t intervalSecs = (uint32_t)interval * 60;
    uint32_t nextSlot     = ((sod / intervalSecs) + 1) * intervalSecs;
    uint32_t secsUntil    = nextSlot - sod;
    nextPublishMs = millis() + secsUntil * 1000UL;
    Serial.printf("[Sched] Next publish in %lus (interval=%dmin)\n",
                  (unsigned long)secsUntil, interval);
}

// ─── MQTT publish ─────────────────────────────────────────────────────────────
void publishMessage(const char* topic, const char* payload) {
    String topicCmd = "AT+CMQTTTOPIC=0," + String(strlen(topic));
    sendATData(topicCmd.c_str(), topic, "OK", 3000);
    String payloadCmd = "AT+CMQTTPAYLOAD=0," + String(strlen(payload));
    sendATData(payloadCmd.c_str(), payload, "OK", 3000);
    String r = sendAT("AT+CMQTTPUB=0,1,60", "+CMQTTPUB: 0,0", 10000);
    if (r.indexOf("+CMQTTPUB: 0,0") >= 0) {
        Serial.println("[MQTT] Published OK");
    } else {
        Serial.println("[MQTT] Publish failed");
        mqttConnected = false;
    }
}

void publishStatus(const char* status) {
    char payload[80];
    snprintf(payload, sizeof(payload), "{\"status\":\"%s\"}", status);
    publishMessage(BASE_TOPIC "/status", payload);
}

void publishData() {
    uint32_t pulses[4];
    noInterrupts();
    for (int i = 0; i < 4; i++) { pulses[i] = pulseCount[i]; pulseCount[i] = 0; }
    interrupts();

    for (int i = 0; i < 4; i++) totalPulses[i] += pulses[i];

    float   pressure    = readPressureBar();
    float   distance    = readSonarCm();
    uint8_t interval    = getActiveInterval();
    float   solarV      = readSolarVoltage();
    int     solarPct    = solarVoltageToPercent(solarV);
    float   batt24V     = readBattery24V();
    int     batt24Pct   = voltage24VToPercent(batt24V);

    checkAlarmConditions(solarV, batt24V);

    const char* alarmType = "none";
    if (solarAlarm && battAlarm) alarmType = "both";
    else if (solarAlarm)         alarmType = "solar_cut";
    else if (battAlarm)          alarmType = "battery_cut";

    char payload[800];
    snprintf(payload, sizeof(payload),
        "{\"flow_1\":%lu,\"flow_2\":%lu,\"flow_3\":%lu,\"flow_4\":%lu,"
        "\"total_1\":%lu,\"total_2\":%lu,\"total_3\":%lu,\"total_4\":%lu,"
        "\"pressure_bar\":%.2f,\"distance_cm\":%.1f,"
        "\"valve_1\":\"%s\",\"valve_2\":\"%s\","
        "\"battery_pct\":%d,\"battery_v\":%.2f,\"power_source\":\"%s\","
        "\"solar_v\":%.2f,\"solar_pct\":%d,"
        "\"battery_24v_v\":%.2f,\"battery_24v_pct\":%d,"
        "\"solar_alarm\":%s,\"battery_alarm\":%s,\"alarm_type\":\"%s\","
        "\"power_state\":\"%s\","
        "\"interval_min\":%d,\"firmware\":\"%s\",\"uptime_s\":%lu}",
        (unsigned long)pulses[0],      (unsigned long)pulses[1],
        (unsigned long)pulses[2],      (unsigned long)pulses[3],
        (unsigned long)totalPulses[0], (unsigned long)totalPulses[1],
        (unsigned long)totalPulses[2], (unsigned long)totalPulses[3],
        pressure, distance,
        valveStateStr(0),
        valveStateStr(1),
        (int)battPct, battVoltage,
        onBackup ? "backup" : "main",
        solarV, solarPct,
        batt24V, batt24Pct,
        solarAlarm ? "true" : "false",
        battAlarm  ? "true" : "false",
        alarmType,
        powerStateStr(),
        interval, FIRMWARE_VER,
        (unsigned long)(millis() / 1000)
    );

    publishMessage(BASE_TOPIC "/data", payload);
}

// ─── Valve control ────────────────────────────────────────────────────────────
const char* valveStateStr(uint8_t idx) {
    switch (valveState[idx]) {
        case VALVE_OPENING: return "opening";
        case VALVE_CLOSING: return "closing";
        default:            return "stopped";
    }
}

// safeSetValve — the ONLY function that may touch valve relay pins.
// CH1=VALVE_1_OPEN(45), CH2=VALVE_1_CLOSE(35), CH3=VALVE_2_OPEN(36), CH4=VALVE_2_CLOSE(37).
// V1 and V2 are independent; their pairs must never be HIGH simultaneously.
void safeSetValve(int valveNum, String action) {
    if (valveNum == 1) {
        if (action == "open") {
            digitalWrite(VALVE_1_CLOSE, LOW);
            delay(200);
            digitalWrite(VALVE_1_OPEN, HIGH);
        } else if (action == "close") {
            digitalWrite(VALVE_1_OPEN, LOW);
            delay(200);
            digitalWrite(VALVE_1_CLOSE, HIGH);
        } else if (action == "stop") {
            digitalWrite(VALVE_1_OPEN,  LOW);
            digitalWrite(VALVE_1_CLOSE, LOW);
        }
    } else if (valveNum == 2) {
        if (action == "open") {
            digitalWrite(VALVE_2_CLOSE, LOW);
            delay(200);
            digitalWrite(VALVE_2_OPEN, HIGH);
        } else if (action == "close") {
            digitalWrite(VALVE_2_OPEN, LOW);
            delay(200);
            digitalWrite(VALVE_2_CLOSE, HIGH);
        } else if (action == "stop") {
            digitalWrite(VALVE_2_OPEN,  LOW);
            digitalWrite(VALVE_2_CLOSE, LOW);
        }
    }
    Serial.printf("[Valve %d] %s\n", valveNum, action.c_str());
}

void handleValveCmd(uint8_t valveIdx, const String& cmd) {
    if (powerState == PWR_INTERNAL_ONLY) {
        Serial.println("[Valve] Blocked — INTERNAL_ONLY mode, valves disabled");
        return;
    }
    String action = "";
    if      (cmd.indexOf("open")  >= 0) action = "open";
    else if (cmd.indexOf("close") >= 0) action = "close";
    else if (cmd.indexOf("stop")  >= 0) action = "stop";
    else return;

    safeSetValve((int)(valveIdx + 1), action);

    if      (action == "open")  valveState[valveIdx] = VALVE_OPENING;
    else if (action == "close") valveState[valveIdx] = VALVE_CLOSING;
    else                        valveState[valveIdx] = VALVE_STOPPED;
}

// ─── Interval command ─────────────────────────────────────────────────────────
void handleIntervalCmd(const String& payload) {
    int val = payload.toInt();
    if (val > 0 && isValidInterval((uint8_t)val)) {
        configuredInterval = (uint8_t)val;
        prefs.putUChar("interval", configuredInterval);
        Serial.printf("[Config] Interval → %d min\n", configuredInterval);
        scheduleNextPublish();
    } else {
        Serial.println("[Config] Invalid interval: " + payload);
    }
}

// ─── Incoming MQTT message parser ─────────────────────────────────────────────
void checkIncoming() {
    if (!modemSerial.available()) return;
    delay(200);
    String raw = "";
    while (modemSerial.available()) raw += (char)modemSerial.read();
    if (raw.length() == 0 || raw.indexOf("+CMQTTRXSTART:") < 0) return;

    Serial.println("[MQTT] Incoming: " + raw);

    int topicIdx   = raw.indexOf("+CMQTTRXTOPIC:");
    int payloadIdx = raw.indexOf("+CMQTTRXPAYLOAD:");
    if (topicIdx < 0 || payloadIdx < 0) return;

    int tLineEnd = raw.indexOf('\n', topicIdx);
    String topic = raw.substring(tLineEnd + 1, raw.indexOf('\n', tLineEnd + 1));
    topic.trim();

    int pLineEnd = raw.indexOf('\n', payloadIdx);
    String payload = raw.substring(pLineEnd + 1);
    int endIdx = payload.indexOf("+CMQTTRXEND:");
    if (endIdx >= 0) payload = payload.substring(0, endIdx);
    payload.trim();

    Serial.println("[MQTT] Topic: " + topic + " | Payload: " + payload);

    if      (topic.endsWith("/valve/1/cmd"))       handleValveCmd(0, payload);
    else if (topic.endsWith("/valve/2/cmd"))       handleValveCmd(1, payload);
    else if (topic.endsWith("/config/interval"))   handleIntervalCmd(payload);
}

// ─── Modem power ──────────────────────────────────────────────────────────────
void modemPowerOn() {
    Serial.println("[Modem] Powering on...");
    pinMode(MODEM_POWER, OUTPUT);
    digitalWrite(MODEM_POWER, HIGH);
    for (int i = 0; i < 80; i++) { delay(100); yield(); }
    modemSerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(1000);
    Serial.println("[Modem] Power on complete");
}

// Graceful software shutdown via AT+CPOF.
// GPIO 33 stays HIGH to avoid cutting board power through the boost circuit.
void modemPowerOff() {
    if (mqttConnected) {
        sendAT("AT+CMQTTDISC=0,60", "OK", 5000);
        delay(500);
        sendAT("AT+CMQTTSTOP", "+CMQTTSTOP", 5000);
    }
    sendAT("AT+CPOF", "OK", 8000);
    delay(1000);
    mqttConnected = false;
    Serial.println("[Modem] Powered off (AT+CPOF)");
}

void modemPowerCycle() {
    Serial.println("[Modem] Cycling power...");
    digitalWrite(MODEM_POWER, LOW);
    delay(3000);
    modemPowerOn();
}

// ─── Load CA cert ─────────────────────────────────────────────────────────────
void loadCACert() {
    Serial.println("[SSL] Loading CA certificate...");
    int certLen = strlen(CA_CERT);
    String cmd = "AT+CCERTDOWN=\"emqx-ca.pem\"," + String(certLen);
    while (modemSerial.available()) modemSerial.read();
    modemSerial.println(cmd);
    Serial.print(">> "); Serial.println(cmd);
    delay(500);
    uint32_t start = millis();
    while (millis() - start < 5000) {
        if (modemSerial.available()) {
            String r = modemSerial.readString();
            if (r.indexOf(">") >= 0) break;
        }
        yield();
    }
    modemSerial.print(CA_CERT);
    Serial.println("[SSL] Cert data sent");
    delay(2000);
    String response = "";
    start = millis();
    while (millis() - start < 5000) {
        while (modemSerial.available()) response += (char)modemSerial.read();
        if (response.indexOf("OK") >= 0) break;
        yield();
    }
    Serial.println("[SSL] Cert response: " + response);
}

// ─── Network init ─────────────────────────────────────────────────────────────
bool networkInit() {
    Serial.print("[Modem] Waiting for AT");
    for (int i = 0; i < 30; i++) {
        if (sendAT("AT", "OK", 1000).indexOf("OK") >= 0) { Serial.println(" OK"); break; }
        if (i == 29) { Serial.println(" FAILED"); return false; }
        Serial.print("."); delay(500);
    }
    sendAT("ATE0",     "OK", 2000);
    sendAT("AT+CMEE=2","OK", 2000);

    Serial.print("[Network] Registering");
    for (int i = 0; i < 30; i++) {
        String r = sendAT("AT+CREG?", "OK", 2000);
        if (r.indexOf(",1") >= 0 || r.indexOf(",5") >= 0) { Serial.println(" OK"); break; }
        if (i == 29) { Serial.println(" FAILED"); return false; }
        Serial.print("."); delay(2000);
    }
    sendAT("AT+CSQ",   "OK", 2000);
    sendAT("AT+COPS?", "OK", 2000);

    sendAT("AT+NETCLOSE", "+NETCLOSE", 15000);
    delay(2000);
    if (sendAT("AT+NETOPEN", "+NETOPEN: 0", 15000).indexOf("+NETOPEN: 0") < 0) {
        Serial.println("[Network] NETOPEN failed"); return false;
    }
    sendAT("AT+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"", "OK", 3000);
    if (sendAT("AT+IPADDR", "+IPADDR", 5000).indexOf("+IPADDR") < 0) {
        Serial.println("[Network] No IP"); return false;
    }

    Serial.println("[DNS] Resolving broker...");
    sendAT("AT+CDNSGIP=\"g7214f51.ala.eu-central-1.emqxsl.com\"", "+CDNSGIP", 10000);

    Serial.println("[NTP] Syncing...");
    sendAT("AT+CNTP=\"pool.ntp.org\",0", "OK", 5000);
    sendAT("AT+CNTP", "+CNTP", 10000);
    String clockResp = sendAT("AT+CCLK?", "OK", 2000);
    ntpSecondsOfDay  = parseClockToSecondsOfDay(clockResp);
    ntpBootMs        = millis();
    Serial.printf("[NTP] Synced: %lus into day (UTC)\n", (unsigned long)ntpSecondsOfDay);

    loadCACert();
    sendAT("AT+CSSLCFG=\"sslversion\",0,4",          "OK", 2000);
    sendAT("AT+CSSLCFG=\"authmode\",0,1",             "OK", 2000);
    sendAT("AT+CSSLCFG=\"ignorelocaltime\",0,1",      "OK", 2000);
    sendAT("AT+CSSLCFG=\"cacert\",0,\"emqx-ca.pem\"", "OK", 2000);
    sendAT("AT+CSSLCFG=\"enableSNI\",0,1",            "OK", 2000);

    Serial.println("[Network] Ready");
    return true;
}

// ─── MQTT connect ─────────────────────────────────────────────────────────────
bool mqttConnect() {
    Serial.println("[Net] Checking connectivity...");
    if (sendAT("AT+CDNSGIP=\"google.com\"", "+CDNSGIP: 1", 10000).indexOf("+CDNSGIP: 1") < 0) {
        Serial.println("[Net] DNS failed — skipping MQTT"); return false;
    }
    Serial.println("[Net] Internet OK");

    Serial.println("[MQTT] Starting...");
    sendAT("AT+CMQTTDISC=0,120", "OK", 15000);
    delay(500);
    sendAT("AT+CMQTTREL=0", "OK", 3000);
    delay(1000);
    sendAT("AT+CMQTTSTOP", "+CMQTTSTOP", 8000);
    delay(3000);

    String r = "";
    for (int i = 0; i < 5; i++) {
        r = sendAT("AT+CMQTTSTART", "+CMQTTSTART: 0", 10000);
        if (r.indexOf("+CMQTTSTART: 0") >= 0) break;
        Serial.println("[MQTT] Start retry " + String(i + 1));
        sendAT("AT+CMQTTSTOP", "+CMQTTSTOP", 8000);
        delay(3000);
    }
    if (r.indexOf("+CMQTTSTART: 0") < 0) { Serial.println("[MQTT] Start failed"); return false; }

    r = sendAT("AT+CMQTTACCQ=0,\"" MQTT_CLIENT "\",1", "OK", 5000);
    if (r.indexOf("OK") < 0) { Serial.println("[MQTT] Acquire failed"); return false; }

    sendAT("AT+CMQTTSSLCFG=0,0", "OK", 2000);

    Serial.println("[MQTT] Connecting...");
    r = sendAT(
        "AT+CMQTTCONNECT=0,\"" MQTT_BROKER "\",60,1,\"" MQTT_USER "\",\"" MQTT_PASS "\"",
        "+CMQTTCONNECT: 0,0", 30000);

    if (r.indexOf("+CMQTTCONNECT: 0,0") < 0) {
        Serial.println("[MQTT] Connect failed"); mqttConnected = false; return false;
    }

    Serial.println("[MQTT] Connected!");
    mqttConnected = true;

    // Wildcard subscription — catches all valve and config commands
    const char* subTopic = BASE_TOPIC "/#";
    String subCmd = "AT+CMQTTSUBTOPIC=0," + String(strlen(subTopic)) + ",2";
    sendATData(subCmd.c_str(), subTopic, "OK", 5000);
    sendAT("AT+CMQTTSUB=0", "+CMQTTSUB: 0,0", 5000);

    publishStatus("online");
    return true;
}

// ─── Critical mode: light sleep with flow ISRs active ────────────────────────
// Enters when 24V battery < 55%. Powers off modem via AT+CPOF (GPIO 33 stays
// HIGH to keep the boost circuit alive). Sleeps in 6-hour intervals; flow ISRs
// on GPIO 21/40/41/39 also wake the CPU for pulse counting then immediately
// re-enters sleep. On each timer wake the modem is brought up, accumulated
// data is published, and the battery level is re-checked.
void runCriticalMode() {
    Serial.println("[Power] CRITICAL — shutting down modem for light sleep");
    if (mqttConnected) publishStatus("external_battery_critical");
    modemPowerOff();

    // Configure flow pins as GPIO wakeup sources (INPUT_PULLUP → active LOW)
    const gpio_num_t flowGPIOs[4] = {GPIO_NUM_21, GPIO_NUM_40, GPIO_NUM_41, GPIO_NUM_39};
    for (int i = 0; i < 4; i++) {
        gpio_wakeup_enable(flowGPIOs[i], GPIO_INTR_LOW_LEVEL);
    }
    esp_sleep_enable_gpio_wakeup();

    while (powerState == PWR_CRITICAL) {
        esp_sleep_enable_timer_wakeup(6ULL * 3600ULL * 1000000ULL);  // 6 hours
        Serial.println("[Sleep] Light sleep (6h timer or flow pulse)");
        Serial.flush();
        esp_light_sleep_start();

        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        if (cause == ESP_SLEEP_WAKEUP_GPIO) {
            // Flow pulse — ISR already counted it, go straight back to sleep
            continue;
        }

        // Timer wake — bring modem online and publish accumulated data
        Serial.println("[Sleep] Timer wake — publishing accumulated data");
        modemPowerOn();

        bool netOk = networkInit();
        if (netOk) mqttConnect();

        checkAlarms();   // updates battAlarm and last24VPct

        if (mqttConnected) {
            publishData();
        }

        if (battAlarm) {
            Serial.println("[Power] 24V gone — switching to INTERNAL_ONLY");
            powerState = PWR_INTERNAL_ONLY;
            // Keep modem on — INTERNAL_ONLY still publishes every 60min
            return;
        }

        if (last24VPct >= 65) {
            Serial.println("[Power] Battery recovered — resuming NORMAL");
            if (mqttConnected) publishStatus("external_battery_recovered");
            powerState = PWR_NORMAL;
            return;
        }

        // Still below threshold — shut modem off and sleep again
        Serial.printf("[Power] Battery at %d%% — back to sleep\n", last24VPct);
        modemPowerOff();
    }
}

// ─── Power management state machine ──────────────────────────────────────────
// Runs on every alarm check cycle (every 2s). Reads last24VPct updated by
// checkAlarms() and drives transitions between power states.
void updatePowerManagement() {
    // PWR_INTERNAL_ONLY: 24V lead-acid physically disconnected (battAlarm set)
    if (battAlarm) {
        if (powerState != PWR_INTERNAL_ONLY) {
            Serial.println("[Power] 24V disconnected — INTERNAL_ONLY mode");
            powerState = PWR_INTERNAL_ONLY;
            // Stop valves — no 24V to operate them safely
            safeSetValve(1, "stop");
            safeSetValve(2, "stop");
            valveState[0] = VALVE_STOPPED;
            valveState[1] = VALVE_STOPPED;
            if (mqttConnected) publishStatus("internal_battery_only");
            scheduleNextPublish();
        }
        return;
    }

    // Recover from INTERNAL_ONLY when 24V comes back
    if (powerState == PWR_INTERNAL_ONLY) {
        Serial.println("[Power] 24V restored — returning to NORMAL");
        powerState = PWR_NORMAL;
        if (mqttConnected) publishStatus("external_battery_recovered");
        scheduleNextPublish();
        return;
    }

    // A valid 24V lead-acid reads >= 21V even when fully dead.
    // Readings below 10V mean the sensor is floating or the battery is
    // physically absent — don't trigger low-battery states in that case.
    if (current24V < 10.0f) {
        if (powerState != PWR_NORMAL) {
            powerState = PWR_NORMAL;
            scheduleNextPublish();
        }
        return;
    }

    // Determine target state from 24V battery percentage
    PowerState target;
    if      (last24VPct >= 65) target = PWR_NORMAL;
    else if (last24VPct >= 55) target = PWR_WARNING;
    else                       target = PWR_CRITICAL;

    if (target == powerState) return;

    powerState = target;
    switch (powerState) {
        case PWR_WARNING:
            Serial.printf("[Power] WARNING — 24V at %d%%\n", last24VPct);
            if (mqttConnected) publishStatus("external_battery_warning");
            scheduleNextPublish();
            break;
        case PWR_CRITICAL:
            Serial.printf("[Power] CRITICAL — 24V at %d%%\n", last24VPct);
            runCriticalMode();     // blocking; updates powerState before returning
            scheduleNextPublish();
            break;
        case PWR_NORMAL:
            Serial.printf("[Power] NORMAL — 24V at %d%%\n", last24VPct);
            if (mqttConnected) publishStatus("external_battery_recovered");
            scheduleNextPublish();
            break;
        default:
            break;
    }
}

// ─── 18650 internal battery monitoring ───────────────────────────────────────
void checkPowerState() {
    bool  wasBak  = onBackup;
    float prevPct = battPct;
    updateBattery();

    if (onBackup && !wasBak) {
        Serial.println("[Power] Switched to backup battery!");
        scheduleNextPublish();
        if (mqttConnected) publishStatus("backup_battery");
    } else if (!onBackup && wasBak) {
        Serial.println("[Power] Main power restored");
        scheduleNextPublish();
        if (mqttConnected) publishStatus("main_power_restored");
    }

    if (!onBackup && battPct < (float)BATT_LOW_PCT && prevPct >= (float)BATT_LOW_PCT) {
        Serial.printf("[Power] Battery low: %.0f%%\n", battPct);
        scheduleNextPublish();
        if (mqttConnected) {
            char msg[48];
            snprintf(msg, sizeof(msg), "battery_low_%.0f_pct", battPct);
            publishStatus(msg);
        }
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    // Hold MODEM_POWER HIGH immediately so the battery boost circuit
    // never sees GPIO 33 floating low during initialisation.
    pinMode(MODEM_POWER, OUTPUT);
    digitalWrite(MODEM_POWER, HIGH);

    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=============================");
    Serial.println(" OURWater Firmware v" FIRMWARE_VER);
    Serial.println("=============================");

    // Flow sensor interrupts
    const uint8_t flowPins[4] = {FLOW_1, FLOW_2, FLOW_3, FLOW_4};
    void (*isrs[4])() = {onFlow1, onFlow2, onFlow3, onFlow4};
    for (int i = 0; i < 4; i++) {
        pinMode(flowPins[i], INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(flowPins[i]), isrs[i], FALLING);
    }
    Serial.println("[GPIO] Flow sensors ready (21/40/41/39)");

    // Valve outputs — all LOW
    const uint8_t valvePins[4] = {VALVE_1_OPEN, VALVE_1_CLOSE, VALVE_2_OPEN, VALVE_2_CLOSE};
    for (int i = 0; i < 4; i++) {
        pinMode(valvePins[i], OUTPUT);
        digitalWrite(valvePins[i], LOW);
    }
    Serial.println("[GPIO] Valve outputs ready (45/35/36/37)");

    // Sonar
    pinMode(SONAR_TRIG, OUTPUT);
    pinMode(SONAR_ECHO, INPUT);
    digitalWrite(SONAR_TRIG, LOW);
    Serial.println("[GPIO] Sonar ready (TRIG=42 ECHO=34)");

    // I2C — MAX17048 battery gauge
    Wire.begin(BATTERY_SDA, BATTERY_SCL);
    Serial.println("[I2C] Bus started (SDA=15 SCL=16)");
    delay(100);
    probeMAX17048();

    // ADC pin config diagnostic
    pinMode(BATTERY_24V_PIN, INPUT);
    gpio_pullup_dis(GPIO_NUM_9); gpio_pulldown_dis(GPIO_NUM_9);
    pinMode(SOLAR_VOLTAGE_PIN, INPUT);
    gpio_pullup_dis(GPIO_NUM_8); gpio_pulldown_dis(GPIO_NUM_8);
    Serial.println("[CAL] IO9 (Battery 24V) raw ADC readings:");
    uint32_t calSum = 0;
    for (int i = 0; i < 10; i++) {
        int raw = analogRead(BATTERY_24V_PIN);
        calSum += raw;
        Serial.printf("[CAL]   sample %d: raw=%d\n", i + 1, raw);
        delay(10);
    }
    int calAvg = (int)(calSum / 10);
    Serial.printf("[CAL] IO9 average raw=%d  (known voltage=25.25V => scale=25.25/(raw*3.3/4095))\n", calAvg);
    float v_pin_cal = calAvg * 3.3f / 4095.0f;
    Serial.printf("[CAL] IO9 v_pin=%.4fV  current_scale=11.0  current_result=%.2fV\n",
                  v_pin_cal, v_pin_cal * 11.0f);

    // Initial 24V battery read to seed power management state before network init
    {
        float v24  = readBattery24V();
        current24V = v24;
        last24VPct = voltage24VToPercent(v24);
        Serial.printf("[Power] Initial 24V: %.2fV (%d%%)%s\n", v24, last24VPct,
                      v24 < 10.0f ? " (no valid reading — power mgmt inactive)" : "");
    }

    // Load persisted interval from flash
    prefs.begin("ourwater", false);
    configuredInterval = prefs.getUChar("interval", 30);
    if (!isValidInterval(configuredInterval)) configuredInterval = 30;
    Serial.printf("[Config] Interval=%dmin  TEST_MODE=%s\n",
                  configuredInterval, TEST_MODE ? "ON" : "OFF");

    modemPowerOn();
    if (!networkInit()) {
        Serial.println("[Boot] Network init failed — will retry in loop");
    } else {
        mqttConnect();
    }
    scheduleNextPublish();
    Serial.println("[Boot] System ready");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    // ── Critical checks always run regardless of MQTT state ──────────────────
    if (millis() - lastBattReadMs >= 1000) {
        lastBattReadMs = millis();
        checkPowerState();   // reads 18650 via MAX17048
    }

    if (millis() - lastAlarmMs >= 2000) {
        lastAlarmMs = millis();
        checkAlarms();             // reads solar & 24V, updates last24VPct + battAlarm
        updatePowerManagement();   // drives PWR_NORMAL/WARNING/CRITICAL/INTERNAL_ONLY
    }

    // ── MQTT reconnect ────────────────────────────────────────────────────────
    if (!mqttConnected) {
        Serial.printf("[MQTT] Disconnected — attempt %d\n", mqttFailCount + 1);

        if (mqttFailCount > 0 && mqttFailCount % 6 == 0) {
            Serial.println("[Modem] 6 consecutive failures — cycling modem power");
            modemPowerCycle();
            if (!networkInit()) {
                Serial.println("[Net] Network failed after modem reset — will keep retrying");
            }
        } else if (mqttFailCount > 0 && mqttFailCount % 3 == 0) {
            Serial.println("[Net] 3 consecutive failures — reinitialising network stack");
            networkInit();
        }

        if (mqttConnect()) {
            mqttFailCount = 0;
            scheduleNextPublish();
        } else {
            mqttFailCount++;
            delay(5000);
        }
        return;
    }

    // ── Normal connected operation ────────────────────────────────────────────
    checkIncoming();

    if (millis() >= nextPublishMs) {
        publishData();
        scheduleNextPublish();
    }

    if (millis() - lastAdcPrintMs >= 5000) {
        lastAdcPrintMs = millis();
        int raw8 = analogRead(SOLAR_VOLTAGE_PIN);
        int raw9 = analogRead(BATTERY_24V_PIN);
        Serial.printf("[ADC] IO8(solar) raw=%d  IO9(batt24v) raw=%d\n", raw8, raw9);
    }

    delay(100);
}
