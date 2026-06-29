# OURWater Firmware

ESP32-S3 firmware for remote water metering nodes. Reads flow sensors, controls motorised valves, monitors solar and battery power, and publishes telemetry to an EMQX MQTT broker over 4G LTE.

---

## Supported Hardware

| Board | Status |
|-------|--------|
| Waveshare ESP32-S3 4G LTE | Confirmed field unit |
| Makerfabs ESP32-S3 4G LTE A7670 | Under test — modem pins unverified |

---

## Quick Start

### Prerequisites

- [Arduino CLI](https://arduino.github.io/arduino-cli/) installed at `C:\Users\erick\arduino-cli\arduino-cli.exe`
- `esp32:esp32` core installed: `arduino-cli core install esp32:esp32`
- Required libraries: `PubSubClient`, `ArduinoJson` (`MAX1704X` is only needed by the main board firmware — not SuperMini)

### 1. Configure the board

Edit `board_config.h` before every flash:

```cpp
#define BOARD_TYPE     BOARD_WAVESHARE_S3     // or BOARD_MAKERFABS_A7670
#define MQTT_CLIENT    "ourwater_001"          // unique per physical unit
#define BASE_TOPIC     "ourwater/ourwater_botswana/site_001_meter_01"
#define TEST_MODE      true                    // false for production (30 min interval)
```

### 2. Find the COM port

```powershell
C:\Users\erick\arduino-cli\arduino-cli.exe board list
```

The ESP32-S3 COM port changes each time Windows re-enumerates the device.

### 3. Kill stale arduino-cli processes

```powershell
Get-Process | Where-Object {$_.Name -match "arduino"} | Stop-Process -Force
```

### 4. Compile

```powershell
C:\Users\erick\arduino-cli\arduino-cli.exe compile `
  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc `
  "C:\Users\erick\OneDrive\Documents\Arduino\libraries\OURWater"
```

> **`CDCOnBoot=cdc` is required.** Without it `Serial` output goes to hardware
> UART0 pins (GPIO43/44), not the USB port — the serial monitor shows nothing.

### 5. Upload

```powershell
C:\Users\erick\arduino-cli\arduino-cli.exe upload `
  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc `
  -p COM6 `
  "C:\Users\erick\OneDrive\Documents\Arduino\libraries\OURWater"
```

Replace `COM6` with the port from step 2.

---

## Repository Layout

```
OURWater/
├── OURWater.ino              Main firmware
├── board_config.h            Per-unit config — edit this before flashing
├── CONTEXT.md                Full developer reference (architecture, pins, MQTT, issues)
├── boards/
│   ├── waveshare_s3.h        Pin definitions — Waveshare board
│   └── makerfabs_a7670.h     Pin definitions — Makerfabs board (under test)
├── OURWater_Expansion/
│   └── OURWater_Expansion.ino  Satellite node firmware (ESP32-S3 Super Mini)
└── OURWater_SuperMini/
    └── OURWater_SuperMini.ino  Mirror of the standalone SuperMini firmware (source of truth: OURWater_SuperMini/)

OURWater_SuperMini/           Standalone WiFi-only node (no LTE modem) — source of truth
├── OURWater_SuperMini.ino
└── board_config.h
```

---

## MQTT Topics

`BASE_TOPIC` example: `ourwater/ourwater_botswana/site_001_meter_01`

| Topic | Direction | Notes |
|-------|-----------|-------|
| `<BASE_TOPIC>/data` | Board → broker | Telemetry JSON every N minutes |
| `<BASE_TOPIC>/status` | Board → broker | `online` / `offline` LWT |
| `<BASE_TOPIC>/valve/1/cmd` | Broker → board | `open` / `close` / `stop` |
| `<BASE_TOPIC>/valve/2/cmd` | Broker → board | `open` / `close` / `stop` |
| `<BASE_TOPIC>/config/interval` | Broker → board | Publish interval in minutes |

---

## SuperMini — Key Facts

| Property | Value |
|----------|-------|
| Board | ESP32-S3 Super Mini |
| Connectivity | WiFi via USB 4G dongle hotspot |
| VALVE_1_OPEN | GPIO **11** |
| VALVE_1_CLOSE | GPIO 12 |
| FLOW_1 | GPIO 10 — ESP-IDF ISR (Arduino `attachInterrupt` silently fails on this variant) |
| Time sync | SNTP on every WiFi connect — `configTime(UTC+2)`, 10 s bounded wait |
| Timestamps | Board sets Botswana local time (UTC+2, no DST); bot.py uses its own `datetime.now(UTC)` for DB rows |
| Publish interval | 30 min normal / 1 min `TEST_MODE=true` — only publishes in `TIER_NORMAL` |
| Dongle cycle | Configurable — `dongle_cycle_interval_min` in Supabase `meters`; only runs in `TIER_NORMAL` |
| Battery tiers | `normal` ≥12.0V / `dark` 11.5–12.0V (offline, schedule local) / `fail_open` <11.5V (valve forced open, close refused) |
| Battery | 12V LiFePO4 4S — no I2C gauge; direct ADC via `BATTERY_24V_PIN` |
| ADC calibration | `BATT_ADC_SCALE` / `SOLAR_ADC_SCALE` default `11.0f` — verify with multimeter: `true_scale = V_multimeter / (pin_mV / 1000)`. `[CAL]` print at boot shows raw mV and reported V. |
| Payload fields | `flow_1`, `valve_1`, `solar_v`, `solar_pct`, `battery_24v_v`, `battery_24v_pct`, `firmware`, `uptime_s` (no `battery_pct` / `battery_v`) |

---

## Known Issues

See [CONTEXT.md — Known Issues](CONTEXT.md#known-issues--todos) for the full list.

**Active investigation:** The Makerfabs A7670 board does not respond to AT commands when powered via USB only. The board was working on USB-only power previously. Suspected causes include modem PWRKEY timing, GPIO33 boot state, or USB current budget. See [CONTEXT.md — Modem USB-Only Power Investigation](CONTEXT.md#modem-usb-only-power-investigation-makerfabs-a7670) for full diagnostic details.

---

## Developer Reference

See **[CONTEXT.md](CONTEXT.md)** for:
- Full pin assignments for all boards
- Power management state machine
- MQTT payload format
- Expansion node UART protocol
- Toolchain commands
- Modem investigation findings
