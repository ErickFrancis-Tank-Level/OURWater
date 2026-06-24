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
| Board FQBN | `esp32:esp32:esp32s3:CDCOnBoot=cdc` |
| Main board COM port | Varies — run `arduino-cli board list` to confirm (has been COM5, COM6, COM7) |
| SuperMini COM port | Separate port — run `arduino-cli board list` to confirm |
| Git | `C:\Program Files\Git\cmd\git.exe` |

> **Important — FQBN must include `CDCOnBoot=cdc`.**  Without it, `Serial` output
> routes to hardware UART0 pins (GPIO43/44), not the USB port — the serial monitor
> shows nothing. This flag is required for all ESP32-S3 boards in this project.

**Compile main firmware:**
```
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc "...\OURWater"
```

**Compile expansion node:**
```
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc "...\OURWater\OURWater_Expansion"
```

**Compile SuperMini standalone:**
```
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc "...\OURWater_SuperMini"
```

**Upload main firmware (replace COMx with actual port):**
```
arduino-cli upload --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p COMx "...\OURWater"
```

**Upload SuperMini (replace COMx with actual port):**
```
arduino-cli upload --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p COMx "...\OURWater_SuperMini"
```

**Before every flash — kill stale arduino-cli processes (PowerShell):**
```powershell
Get-Process | Where-Object {$_.Name -match "arduino"} | Stop-Process -Force
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
| **Makerfabs modem silent on USB-only power — under investigation** | Open — see section below |
| Standalone SuperMini firmware | Fixed — SM-1.0.0 written and deployed on COM6, serial OW-SM-001. See `OURWater_SuperMini/` folder. |
| `attachInterrupt()` silent failure on ESP32-S3 Super Mini | Fixed — ISR never fires with Arduino API on this variant. Fix: ESP-IDF `gpio_install_isr_service(0)` + `gpio_isr_handler_add()` with `void IRAM_ATTR handler(void* arg)` |
| EMQX Serverless TLS CA mismatch | Fixed — EMQX Serverless uses its own CA; `setCACert(DigiCert)` fails. Fix: `secureClient.setInsecure()` (still TLS-encrypted) |

---

## Modem USB-Only Power Investigation (Makerfabs A7670)

**Status: unresolved — needs senior developer review**

### Background

The Makerfabs ESP32-S3 4G LTE A7670 board was confirmed **working on USB power alone** (no 24V battery) in at least one prior session. After firmware changes in commit `6a4b9ad` (which reduced the modem cold-boot wait from ~7 s to 2.3 s and enabled CDCOnBoot), the modem stopped responding entirely.

### What was tried

All of the following produced **zero bytes from the modem** across multiple board power-cycles and USB replug cycles:

| Test | Result |
|------|--------|
| Original MODEM_RX=17, MODEM_TX=18 at 115200 baud | 0 bytes |
| Swapped MODEM_RX=18, MODEM_TX=17 | 0 bytes |
| Baud scan: 9600, 19200, 57600, 115200, 230400 | 0 bytes at all rates |
| Full GPIO RX scan: every safe GPIO 0–48 (excl. 19,20,43,44,45,46) as RX with TX=17 | 0 bytes on all |
| Full GPIO power pin scan: GPIOs 4,5,10,14,21,33,47,48 as modem power/PWRKEY | 0 bytes on all |
| 500ms LOW → 8s HIGH cycle on MODEM_POWER before AT commands | 0 bytes |

### Current `modemPowerOn()` implementation

```cpp
void modemPowerOn() {
    Serial.println("[Modem] Powering on...");
    modemSerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    // LOW→HIGH rising edge triggers the modem auto-start circuit.
    digitalWrite(MODEM_POWER, LOW);
    delay(500);
    digitalWrite(MODEM_POWER, HIGH);
    for (int i = 0; i < 80; i++) { delay(100); yield(); }   // 8s cold-boot
    Serial.println("[Modem] Power on complete");
}
```

`setup()` asserts `MODEM_POWER HIGH` before anything else so the 24V boost
circuit is never left floating. `modemPowerOn()` then pulls it LOW to create
the rising edge the A7670 PWRKEY auto-start circuit requires.

### Hypotheses for senior dev

1. **PWRKEY circuit timing**: The A7670 PWRKEY pin has different hold-time requirements for power-on vs power-off. If `MODEM_POWER` (GPIO33) maps to both VBAT-enable and PWRKEY simultaneously, a 3s LOW may trigger a power-off rather than a reset. The schematic for the Makerfabs PCB has **not been obtained or verified** — the header file says "modem pins not yet confirmed against schematic".

2. **Firmware regression in `6a4b9ad`**: That commit reduced `modemPowerOn()` wait from ~7 s to 2.3 s. Sending AT commands during A7670 boot can leave the modem in a silent unresponsive state that persists across soft resets. A full board power-cycle (USB unplug) should clear this — but if the module has a supercap that holds charge, it may not.

3. **CDCOnBoot interaction**: `CDCOnBoot=cdc` enables the USB CDC stack early in boot. It's possible this creates brief noise on GPIO pins the A7670 interprets as PWRKEY activity. This is speculative.

4. **GPIO33 behaviour at boot**: ESP32-S3 GPIO33 may be driven HIGH by the chip's internal pull during POR (power-on reset), which could suppress the PWRKEY rising-edge that the A7670 needs. The `setup()` `HIGH` guard was added to prevent a different issue (boost circuit floating) and may have introduced this.

5. **USB-only current budget**: The A7670 peak transmit current is ~2A. USB 2.0 provides 500mA; USB 3.0 provides 900mA. If the modem is drawing more than the port allows, it may boot and immediately shut down. The board supposedly worked previously — so this may be a marginal or intermittent issue.

### Recommended next steps

- Obtain and verify the Makerfabs A7670 board schematic — specifically which ESP32 GPIOs connect to A7670 PWRKEY, VBAT/VDDEXT, and STATUS
- Measure GPIO33 voltage with a multimeter at boot (before `setup()` runs — power on with no firmware or a bare blink sketch)
- Use a USB power meter to measure current draw when modem is supposed to start
- Try `modemPowerOn()` with a 2.5s LOW pulse (matches SIM7600 PWRKEY spec exactly) instead of 500ms
- Test with a fresh flash of the **last known working firmware state** (before `6a4b9ad`) to isolate whether this is a firmware or hardware regression
