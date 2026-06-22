// =============================================================================
//  OURWater Expansion Node — ESP32-S3 Super Mini
//  Satellite device connected to the main board via UART.
//  Counts pulses on two flow sensors, reads one pressure transducer,
//  and controls one motorised valve pair.
//  Sleeps between wake cycles; the main board drives timing via WAKE_PIN.
// =============================================================================

#include "driver/gpio.h"
#include "esp_sleep.h"

// ─── Pin Definitions ──────────────────────────────────────────────────────────
#define WAKE_PIN          4   // INPUT — main board pulls HIGH to start a cycle
#define UART_RX          20
#define UART_TX          21
#define FLOW_EXP_1        5   // INPUT_PULLUP, FALLING edge ISR
#define FLOW_EXP_2        6   // INPUT_PULLUP, FALLING edge ISR
#define VALVE_EXP_OPEN    7   // OUTPUT — relay: energise to open valve
#define VALVE_EXP_CLOSE   8   // OUTPUT — relay: energise to close valve
#define PRESSURE_EXP      1   // ADC1 ch0 — 4-20 mA transducer, 165 Ω shunt

// ─── Config ───────────────────────────────────────────────────────────────────
#define BAUD              115200
#define CMD_TIMEOUT_MS      500   // wait this long for a command after sending payload
#define PRESSURE_MAX_BAR   10.0f  // full-scale of the 4-20 mA transducer

// ─── State ────────────────────────────────────────────────────────────────────
HardwareSerial expSerial(1);

volatile uint32_t pulseCount[2] = {0, 0};
volatile uint32_t lastFlowMs[2] = {0, 0};

enum ValveState : uint8_t { VALVE_STOPPED, VALVE_OPEN, VALVE_CLOSED };
ValveState valveState = VALVE_STOPPED;

// ─── Flow ISRs ────────────────────────────────────────────────────────────────
void IRAM_ATTR onFlow1() {
    uint32_t n = millis();
    if (n - lastFlowMs[0] > 50) { pulseCount[0]++; lastFlowMs[0] = n; }
}

void IRAM_ATTR onFlow2() {
    uint32_t n = millis();
    if (n - lastFlowMs[1] > 50) { pulseCount[1]++; lastFlowMs[1] = n; }
}

// ─── Pressure ─────────────────────────────────────────────────────────────────
float readPressure() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += analogRead(PRESSURE_EXP);
    float voltage   = (float)(sum / 16) / 4095.0f * 3.3f;
    float currentMA = voltage / 165.0f * 1000.0f;   // 165 Ω shunt: I = V/R
    float bar       = (currentMA - 4.0f) / 16.0f * PRESSURE_MAX_BAR;
    return constrain(bar, 0.0f, PRESSURE_MAX_BAR);
}

// ─── Valve ────────────────────────────────────────────────────────────────────
const char* valveStateStr() {
    switch (valveState) {
        case VALVE_OPEN:   return "open";
        case VALVE_CLOSED: return "closed";
        default:           return "stopped";
    }
}

// Never energise both pins of the pair simultaneously.
void setValve(const char* action) {
    if (strcmp(action, "open") == 0) {
        digitalWrite(VALVE_EXP_CLOSE, LOW);
        delay(200);
        digitalWrite(VALVE_EXP_OPEN, HIGH);
        valveState = VALVE_OPEN;
    } else if (strcmp(action, "close") == 0) {
        digitalWrite(VALVE_EXP_OPEN, LOW);
        delay(200);
        digitalWrite(VALVE_EXP_CLOSE, HIGH);
        valveState = VALVE_CLOSED;
    } else if (strcmp(action, "stop") == 0) {
        digitalWrite(VALVE_EXP_OPEN,  LOW);
        digitalWrite(VALVE_EXP_CLOSE, LOW);
        valveState = VALVE_STOPPED;
    }
    Serial.printf("[Valve] %s\n", action);
}

// ─── UART communication ───────────────────────────────────────────────────────
void sendPayload(uint32_t f1, uint32_t f2) {
    float pressure = readPressure();
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"flow_exp_1\":%lu,\"flow_exp_2\":%lu,"
        "\"pressure_exp\":%.2f,\"valve_exp\":\"%s\"}\n",
        (unsigned long)f1, (unsigned long)f2,
        pressure, valveStateStr());
    expSerial.print(buf);
    Serial.print("[TX] "); Serial.print(buf);
}

void handleCommand(const String& raw) {
    if      (raw.indexOf("\"open\"")  >= 0) setValve("open");
    else if (raw.indexOf("\"close\"") >= 0) setValve("close");
    else if (raw.indexOf("\"stop\"")  >= 0) setValve("stop");
    else { Serial.println("[RX] Unknown command — ignored"); return; }
    expSerial.println("{\"ack\":true}");
    Serial.println("[TX] {\"ack\":true}");
}

// ─── Sleep ────────────────────────────────────────────────────────────────────
void configureSleepWakeups() {
    // WAKE_PIN HIGH: main board is requesting a data-collection cycle
    gpio_wakeup_enable((gpio_num_t)WAKE_PIN, GPIO_INTR_HIGH_LEVEL);
    // Flow pins LOW (INPUT_PULLUP): sensor pulse wakes CPU briefly so ISR can count
    gpio_wakeup_enable((gpio_num_t)FLOW_EXP_1, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)FLOW_EXP_2, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
}

void enterSleep() {
    Serial.println("[Sleep] Entering light sleep");
    Serial.flush();
    esp_light_sleep_start();
    // Execution resumes here on wakeup
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== OURWater Expansion Node ===");

    // Flow sensors
    pinMode(FLOW_EXP_1, INPUT_PULLUP);
    pinMode(FLOW_EXP_2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_EXP_1), onFlow1, FALLING);
    attachInterrupt(digitalPinToInterrupt(FLOW_EXP_2), onFlow2, FALLING);
    Serial.println("[GPIO] Flow sensors ready (IO5, IO6)");

    // Valve outputs — both LOW (neither relay energised)
    pinMode(VALVE_EXP_OPEN,  OUTPUT); digitalWrite(VALVE_EXP_OPEN,  LOW);
    pinMode(VALVE_EXP_CLOSE, OUTPUT); digitalWrite(VALVE_EXP_CLOSE, LOW);
    Serial.println("[GPIO] Valve outputs ready (IO7 open, IO8 close)");

    // Wake pin
    pinMode(WAKE_PIN, INPUT);
    Serial.println("[GPIO] Wake pin ready (IO4)");

    // UART to main board
    expSerial.begin(BAUD, SERIAL_8N1, UART_RX, UART_TX);
    Serial.println("[UART] Ready (RX=IO20 TX=IO21 115200 baud)");

    configureSleepWakeups();
    enterSleep();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    // ── Flow pulse wake: ISR already counted it — go straight back to sleep ──
    if (digitalRead(WAKE_PIN) == LOW) {
        enterSleep();
        return;
    }

    // ── WAKE_PIN HIGH: data-collection cycle ──────────────────────────────────
    Serial.println("[Wake] Main board request");

    // Snapshot and reset pulse counters atomically
    uint32_t f1, f2;
    noInterrupts();
    f1 = pulseCount[0]; pulseCount[0] = 0;
    f2 = pulseCount[1]; pulseCount[1] = 0;
    interrupts();

    sendPayload(f1, f2);

    // Listen for an optional valve command
    String cmd = "";
    uint32_t deadline = millis() + CMD_TIMEOUT_MS;
    while (millis() < deadline) {
        while (expSerial.available()) cmd += (char)expSerial.read();
        if (cmd.length() > 0 && cmd.indexOf('}') >= 0) break;
        delay(10);
    }

    if (cmd.length() > 0) {
        Serial.println("[RX] " + cmd);
        handleCommand(cmd);
    } else {
        Serial.println("[RX] No command within timeout");
    }

    // Wait for main board to release WAKE_PIN before sleeping
    while (digitalRead(WAKE_PIN) == HIGH) delay(10);
    Serial.println("[Wake] WAKE_PIN released — sleeping");
    enterSleep();
}
