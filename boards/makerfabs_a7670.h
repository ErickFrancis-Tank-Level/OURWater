#pragma once
// =============================================================================
//  Board: Makerfabs ESP32-S3 4G LTE A7670
//  Status: under test — modem pins not yet confirmed against schematic
// =============================================================================

#define BOARD_LABEL "Makerfabs ESP32-S3 4G LTE A7670"

// -----------------------------------------------------------------------------
//  LTE Modem
//  TODO: verify modem pins against Makerfabs schematic before field deployment
// -----------------------------------------------------------------------------
#define MODEM_POWER       33
#define MODEM_RX          17
#define MODEM_TX          18

// -----------------------------------------------------------------------------
//  Flow sensors  (INPUT_PULLUP, FALLING edge ISR)
//  Avoids strapping pins (0, 3, 45, 46) and internal USB (19, 20 on S3).
// -----------------------------------------------------------------------------
#define FLOW_1             4
#define FLOW_2             5
#define FLOW_3            12   // TODO: assign when 3rd sensor is wired
#define FLOW_4            13   // TODO: assign when 4th sensor is wired

// -----------------------------------------------------------------------------
//  Motorised valve relay pairs
//  Never energise both pins of a pair simultaneously.
// -----------------------------------------------------------------------------
#define VALVE_1_OPEN       6
#define VALVE_1_CLOSE      7
#define VALVE_2_OPEN      15
#define VALVE_2_CLOSE     16

// -----------------------------------------------------------------------------
//  ADC voltage sense  (ADC1 channels — safe with LTE active)
// -----------------------------------------------------------------------------
#define SOLAR_VOLTAGE_PIN  1
#define BATTERY_24V_PIN    2

// -----------------------------------------------------------------------------
//  MAX17048 fuel gauge (18650 internal battery) — I2C
// -----------------------------------------------------------------------------
#define BATTERY_SDA       41
#define BATTERY_SCL       42

// -----------------------------------------------------------------------------
//  Expansion UART
// -----------------------------------------------------------------------------
#define EXPANSION_RX      38
#define EXPANSION_TX      39
#define EXPANSION_WAKE    40

// -----------------------------------------------------------------------------
//  Feature flags
// -----------------------------------------------------------------------------
#define HAS_PRESSURE      0   // no pressure transducer on this board
#define HAS_SONAR         0   // no sonar on this board
#define HAS_EXPANSION     0

// -----------------------------------------------------------------------------
//  Calibration
//  ADC divider values are TBD — using waveshare values as starting point.
//  TODO: measure actual resistors on Makerfabs PCB and update scales.
// -----------------------------------------------------------------------------
#define CUBIC_METRES_PER_PULSE   0.001f

#define VALVE_PULSE_MS           500

#define SOLAR_ADC_SCALE          28.1f    // TODO: verify divider on this PCB
#define BATT24V_ADC_SCALE        11.0f    // TODO: verify divider on this PCB

#define BATT24V_FULL_V           25.6f
#define BATT24V_EMPTY_V          21.0f

#define SOLAR_MAX_V              40.0f
#define BATT_LOW_PCT             70
