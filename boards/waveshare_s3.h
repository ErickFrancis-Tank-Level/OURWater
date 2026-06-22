#pragma once
// =============================================================================
//  Board: Waveshare ESP32-S3 4G LTE
//  Status: confirmed working in field (OW-0001, Botswana site 001)
// =============================================================================

#define BOARD_LABEL "Waveshare ESP32-S3 4G LTE"

// -----------------------------------------------------------------------------
//  LTE Modem (SIM7600 / A7670 via UART1)
// -----------------------------------------------------------------------------
#define MODEM_POWER       33   // also gates 24V boost circuit — keep HIGH
#define MODEM_RX          17
#define MODEM_TX          18

// -----------------------------------------------------------------------------
//  Flow sensors  (INPUT_PULLUP, FALLING edge ISR)
// -----------------------------------------------------------------------------
#define FLOW_1            21
#define FLOW_2            40
#define FLOW_3            41
#define FLOW_4            39

// -----------------------------------------------------------------------------
//  Motorised valve relay pairs
//  Never energise both pins of a pair simultaneously.
// -----------------------------------------------------------------------------
#define VALVE_1_OPEN      45
#define VALVE_1_CLOSE     35
#define VALVE_2_OPEN      36
#define VALVE_2_CLOSE     37

// -----------------------------------------------------------------------------
//  Sensors
// -----------------------------------------------------------------------------
#define PRESSURE_PIN       1   // 4-20 mA transducer, 165 Ω shunt → ADC
#define SONAR_TRIG        42
#define SONAR_ECHO        34

// -----------------------------------------------------------------------------
//  MAX17048 fuel gauge (18650 internal battery) — I2C
// -----------------------------------------------------------------------------
#define BATTERY_SDA       15
#define BATTERY_SCL       16

// -----------------------------------------------------------------------------
//  ADC voltage sense
// -----------------------------------------------------------------------------
#define SOLAR_VOLTAGE_PIN  8   // solar panel: 220k+51k / 10k divider → ×28.1
#define BATTERY_24V_PIN    9   // 24V lead-acid: 100k / 10k divider → ×11.0

// -----------------------------------------------------------------------------
//  Expansion UART  (Super Mini slave connection — future use)
// -----------------------------------------------------------------------------
#define EXPANSION_RX      13
#define EXPANSION_TX      14
#define EXPANSION_WAKE    48

// -----------------------------------------------------------------------------
//  Feature flags
// -----------------------------------------------------------------------------
#define HAS_PRESSURE      1
#define HAS_SONAR         1
#define HAS_EXPANSION     0   // set 1 when Super Mini slave is wired

// -----------------------------------------------------------------------------
//  Calibration
// -----------------------------------------------------------------------------
#define CUBIC_METRES_PER_PULSE   0.001f   // YF-S201: 1 pulse = 1 L

#define PRESSURE_MAX_BAR         10.0f    // 4-20 mA transducer full-scale
#define VALVE_PULSE_MS           500      // relay energise duration (ms)

#define SOLAR_ADC_SCALE          28.1f    // (220k+51k+10k)/10k
#define BATT24V_ADC_SCALE        11.0f    // (100k+10k)/10k

#define BATT24V_FULL_V           25.6f    // lead-acid 100 %
#define BATT24V_EMPTY_V          21.0f    // lead-acid   0 %

#define SOLAR_MAX_V              40.0f    // 2 panels in series peak OCV
#define BATT_LOW_PCT             70       // 18650 low-battery alert threshold
