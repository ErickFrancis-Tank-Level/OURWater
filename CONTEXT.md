# OURWater — Firmware Context

> **Share this file with any developer or Claude instance working on the firmware.**
> It covers file structure, board abstraction, pin assignments, power management,
> MQTT protocol, and the expansion node.

---

## Repository Layout

```
OURWater/                                ← This repo (github: OURWater)
├── board_config.h                       ← Edit this when flashing a new board
├── OURWater.ino                         ← Main firmware — do not edit for new boards
├── boards/
│   ├── waveshare_s3.h                   ← Waveshare ESP32-S3 4G LTE (confirmed field unit)
│   └── makerfabs_a7670.h               ← Makerfabs ESP32-S3 4G LTE A7670 (under test)
└── OURWater_Expansion/
    └── OURWater_Expansion.ino           ← Super Mini satellite node (separate flash target)

OURWater_SuperMini/                      ← Separate library folder (same repo root)
├── board_config.h                       ← Per-board WiFi/MQTT/Supabase credentials
└── OURWater_SuperMini.ino               ← Standalone WiFi-only node SM-1.0.0
```

---

## Board Abstraction

`board_config.h` is the **only file that changes between units**. It:

1. Sets `BOARD_TYPE` to select the hardware variant
2. Includes the matching board header from `boards/`
3. Holds per-unit identity (`MQTT_CLIENT`, `BASE_TOPIC`, `TEST_MODE`)

`OURWater.ino` reads all pin and calibration defines from whichever board header
was included. Feature flags (`HAS_PRESSURE`, `HAS_SONAR`) gate code sections with
`#if` so unsupported peripherals are compiled out entirely.

### Adding a new board

1. Create `boards/myboard.h` — define all pins, calibration constants, and feature flags
2. Add `#define BOARD_MYBOARD 3` and a matching `#elif` block in `board_config.h`
3. Set `BOARD_TYPE BOARD_MYBOARD` and flash

### Defines every board header must supply

| Define | Type | Notes |
|--------|------|-------|
| `BOARD_LABEL` | string | Printed at boot |
| `MODEM_POWER` | pin | Also gates 24V boost circuit — keep HIGH |
| `MODEM_RX`, `MODEM_TX` | pins | UART1 to SIM modem |
| `FLOW_1..4` | pins | INPUT_PULLUP, FALLING edge ISR |
| `VALVE_1_OPEN/CLOSE` | pins | Relay pair — never both HIGH |
| `VALVE_2_OPEN/CLOSE` | pins | Relay pair — never both HIGH |
| `BATTERY_SDA`, `BATTERY_SCL` | pins | MAX17048 I2C |
| `SOLAR_VOLTAGE_PIN` | ADC1 pin | Voltage divider |
| `BATTERY_24V_PIN` | ADC1 pin | Voltage divider |
| `EXPANSION_RX/TX/WAKE` | pins | UART to Super Mini |
| `HAS_PRESSURE` | 0 or 1 | Enables `readPressureBar()` |
| `HAS_SONAR` | 0 or 1 | Enables `readSonarCm()` |
| `HAS_EXPANSION` | 0 or 1 | Reserved — not yet wired |
| `CUBIC_METRES_PER_PULSE` | float | e.g. `0.001f` (1 L/pulse) |
| `VALVE_PULSE_MS` | int | Relay energise duration |
| `SOLAR_ADC_SCALE` | float | Divider ratio (top+bot)/bot |
| `BATT24V_ADC_SCALE` | float | Divider ratio |
| `BATT24V_FULL_V` | float | Lead-acid 100% voltage |
| `BATT24V_EMPTY_V` | float | Lead-acid 0% voltage |
| `SOLAR_MAX_V` | float | Panel peak OCV |
| `BATT_LOW_PCT` | int | 18650 low-battery threshold % |

If `HAS_PRESSURE = 1`, also supply: `PRESSURE_PIN`, `PRESSURE_MAX_BAR`
If `HAS_SONAR = 1`, also supply: `SONAR_TRIG`, `SONAR_ECHO`

---

## Board: Waveshare ESP32-S3 4G LTE (`boards/waveshare_s3.h`)

Status: **confirmed working in field** (OW-0001, Botswana site 001)

```
MODEM_POWER = 33     also gates 24V boost circuit — keep HIGH
MODEM_RX    = 17
MODEM_TX    = 18

FLOW_1 = 21   FLOW_2 = 40   FLOW_3 = 41   FLOW_4 = 39

VALVE_1_OPEN = 45   VALVE_1_CLOSE = 35
VALVE_2_OPEN = 36   VALVE_2_CLOSE = 37

PRESSURE_PIN   = 1    4-20 mA transducer, 165 Ω shunt → ADC1
SONAR_TRIG     = 42
SONAR_ECHO     = 34

BATTERY_SDA    = 15   BATTERY_SCL = 16   (MAX17048 I2C)
SOLAR_VOLTAGE_PIN = 8    divider: (220k+51k+10k)/10k → ×28.1
BATTERY_24V_PIN   = 9    divider: (100k+10k)/10k     → ×11.0

EXPANSION_RX = 13   EXPANSION_TX = 14   EXPANSION_WAKE = 48

HAS_PRESSURE = 1   HAS_SONAR = 1   HAS_EXPANSION = 0
```

---

## Board: Makerfabs ESP32-S3 4G LTE A7670 (`boards/makerfabs_a7670.h`)

Status: **under test** — modem pins not yet confirmed against schematic; ADC
divider values copied from Waveshare (TODO: measure actual PCB resistors)

```
MODEM_POWER = 33
MODEM_RX    = 17
MODEM_TX    = 18

FLOW_1 =  4   FLOW_2 =  5
FLOW_3 = 12   FLOW_4 = 13   (assigned when 3rd/4th sensors wired)

VALVE_1_OPEN =  6   VALVE_1_CLOSE =  7
VALVE_2_OPEN = 15   VALVE_2_CLOSE = 16

BATTERY_SDA = 41   BATTERY_SCL = 42   (MAX17048 I2C)
SOLAR_VOLTAGE_PIN =  1
BATTERY_24V_PIN   =  2

EXPANSION_RX = 38   EXPANSION_TX = 39   EXPANSION_WAKE = 40

HAS_PRESSURE = 0   HAS_SONAR = 0   HAS_EXPANSION = 0
```

---

## Expansion Node (`OURWater_Expansion/OURWater_Expansion.ino`)

Runs on an **ESP32-S3 Super Mini** wired to the main board. No WiFi, no MQTT —
UART only at 115200 baud.

### Pin Assignments

```
WAKE_PIN        =  4   INPUT — main board pulls HIGH to start a cycle
UART_RX         = 20
UART_TX         = 21
FLOW_EXP_1      =  5   INPUT_PULLUP, FALLING edge ISR
FLOW_EXP_2      =  6   INPUT_PULLUP, FALLING edge ISR
VALVE_EXP_OPEN  =  7   OUTPUT relay — open
VALVE_EXP_CLOSE =  8   OUTPUT relay — close
PRESSURE_EXP    =  1   ADC1 ch0 — 4-20 mA transducer, 165 Ω shunt
```

### Wake Cycle

1. Node sleeps in ESP32 light sleep
2. **Flow pulse** (IO5 or IO6 LOW) → GPIO wakeup → ISR counts → back to sleep.
   Pulses accumulate during sleep.
3. **Main board pulls WAKE_PIN HIGH** → full wake
4. Node sends JSON payload; resets counters
5. Waits up to 500 ms for an optional valve command
6. Executes command; sends `{"ack":true}`
7. Waits for WAKE_PIN to go LOW → sleeps

### UART Protocol (newline-delimited JSON)

**Node → Main (on every wake):**
```json
{"flow_exp_1": 12, "flow_exp_2": 0, "pressure_exp": 1.45, "valve_exp": "open"}
```

**Main → Node (optional, within 500 ms):**
```json
{"valve_exp": "open|close|stop"}
```

**Node → Main (after executing command):**
```json
{"ack": true}
```

Valve states reported: `"open"`, `"closed"`, `"stopped"`

---

## Power Management States (main board)

| State | Condition | Behaviour |
|-------|-----------|-----------|
| `PWR_NORMAL` | 24V ≥ 65% | Full operation, configured interval |
| `PWR_WARNING` | 24V 55–64% | Publishes alarm, forces ≥ 60 min interval |
| `PWR_CRITICAL` | 24V < 55% | `AT+CPOF` modem off (GPIO 33 stays HIGH), ESP32 light sleep; flow ISRs active; wakes every 6 h to publish + re-check |
| `PWR_INTERNAL_ONLY` | `battAlarm=true` (24V cable cut) | Valves disabled, 60 min interval, modem stays on |

**Guard:** if `current24V < 10.0V` the ADC is floating — power management is
bypassed and the board stays in NORMAL.

**24V battery % formula:** `(V − 21.0) / (25.6 − 21.0) × 100`, clamped 0–100

---

## MQTT Topics

| Topic | Direction | QoS | Notes |
|-------|-----------|-----|-------|
| `<BASE_TOPIC>/data` | Board → broker | 1 | Main telemetry JSON |
| `<BASE_TOPIC>/status` | Board → broker | 1 | e.g. `online`, `offline` (LWT) |
| `<BASE_TOPIC>/valve/1/cmd` | Broker → board | 2 sub | `open/close/stop`, retained |
| `<BASE_TOPIC>/valve/2/cmd` | Broker → board | 2 sub | retained |
| `<BASE_TOPIC>/config/interval` | Broker → board | 2 sub | Publish interval in minutes |

`BASE_TOPIC` example: `ourwater/ourwater_botswana/site_001_meter_01`

Board subscribes to `BASE_TOPIC/#` (wildcard, QoS 2).
Bot subscribes to `ourwater/#` (wildcard, QoS 1).

---

## Toolchain

| Tool | Path |
|------|------|
| Arduino CLI | `C:\Users\erick\arduino-cli\arduino-cli.exe` |
| Board FQBN | `esp32:esp32:esp32s3` |
| COM5 | Main boards (Waveshare/Makerfabs) — LTE modem + expansion |
| COM6 | SuperMini standalone (OURWater_SuperMini.ino) |
| Git | `C:\Program Files\Git\cmd\git.exe` |

**Compile main firmware:**
```
arduino-cli compile --fqbn esp32:esp32:esp32s3 "...\OURWater"
```

**Compile expansion node:**
```
arduino-cli compile --fqbn esp32:esp32:esp32s3 "...\OURWater\OURWater_Expansion"
```

**Compile SuperMini standalone:**
```
arduino-cli compile --fqbn esp32:esp32:esp32s3 "...\OURWater_SuperMini"
```

**Upload main firmware:**
```
arduino-cli upload --fqbn esp32:esp32:esp32s3 -p COM5 "...\OURWater"
```

**Upload SuperMini:**
```
arduino-cli upload --fqbn esp32:esp32:esp32s3 -p COM6 "...\OURWater_SuperMini"
```

**Kill stale arduino-cli before flash:**
```
taskkill /F /IM arduino-cli.exe
```

---

## SuperMini Standalone Firmware (`OURWater_SuperMini/`)

A separate firmware for ESP32-S3 Super Mini boards deployed **without** a main board — WiFi only via USB 4G dongle hotspot, no LTE modem.

### Pin Assignments

```
FLOW_1_PIN        = 10   INPUT_PULLUP — ESP-IDF ISR NEGEDGE (not Arduino attachInterrupt)
VALVE_1_OPEN      = 11   OUTPUT relay — open
VALVE_1_CLOSE     = 12   OUTPUT relay — close
SOLAR_VOLTAGE_PIN =  3   ADC
BATTERY_24V_PIN   =  4   ADC
DONGLE_BUTTON_PIN =  5   OUTPUT — transistor drives USB dongle power button
STATUS_LED_PIN    = 13   OUTPUT
BATTERY_SDA       =  6   I2C SDA — MAX17048
BATTERY_SCL       =  7   I2C SCL — MAX17048
```

### Key Differences from Main Firmware

| | OURWater.ino | OURWater_SuperMini.ino |
|---|---|---|
| Connectivity | LTE SIM modem (UART) | WiFi via USB 4G dongle |
| Dongle management | None | Power-cycles dongle via GPIO5 transistor |
| Flow ISR | Arduino `attachInterrupt()` | ESP-IDF `gpio_isr_handler_add()` — Arduino API silently fails on this variant |
| TLS | `setCACert(CA_CERT)` | `setInsecure()` — EMQX Serverless uses own CA |
| `total_1` | Published (cumulative) | Not published — only `flow_1` (interval delta) |
| Publish interval | Configurable via MQTT | 30 min (TEST_MODE=false) / 1 min (TEST_MODE=true) |
| Dongle cycle | — | Configurable via Supabase `dongle_cycle_interval_min` |

### Payload Fields

`flow_1`, `valve_1`, `battery_pct`, `battery_v`, `solar_v`, `solar_pct`, `battery_24v_v`, `battery_24v_pct`, `firmware`, `uptime_s`

### `board_config.h` (per-unit file, edit before flash)

```cpp
#define WIFI_SSID      "..."
#define WIFI_PASS      "..."
#define MQTT_CLIENT    "ourwater_sm_001"   // unique per board
#define BASE_TOPIC     "ourwater/<client>/<site>"
#define SERIAL_NO      "OW-SM-001"
#define TEST_MODE      false               // true = 1 min publish, false = 30 min
// MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASS — shared, do not change
// SUPABASE_URL, SUPABASE_KEY — for fetching dongle timing on boot
```

---

## Known Issues / TODOs

| Item | Status |
|------|--------|
| MAX17048 (18650 gauge) returning NACK on both I2C addresses | Open — likely wiring |
| Makerfabs modem pins not confirmed against schematic | TODO before field deployment |
| Makerfabs ADC divider scales copied from Waveshare | TODO — measure actual PCB resistors |
| `HAS_EXPANSION` not yet wired on either board | Future — main board UART to Super Mini not connected |
| Standalone SuperMini firmware | Fixed — SM-1.0.0 written and deployed on COM6, serial OW-SM-001. See `OURWater_SuperMini/` folder. |
| `attachInterrupt()` silent failure on ESP32-S3 Super Mini | Fixed — ISR never fires with Arduino API on this variant. Fix: ESP-IDF `gpio_install_isr_service(0)` + `gpio_isr_handler_add()` with `void IRAM_ATTR handler(void* arg)` |
| EMQX Serverless TLS CA mismatch | Fixed — EMQX Serverless uses its own CA; `setCACert(DigiCert)` fails. Fix: `secureClient.setInsecure()` (still TLS-encrypted) |
