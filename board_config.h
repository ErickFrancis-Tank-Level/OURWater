// =============================================================================
//  OURWater Board Configuration
//  Edit ONLY this file when flashing a new board.
//  OURWater.ino and the boards/ headers stay the same.
// =============================================================================

// -----------------------------------------------------------------------------
//  BOARD SELECTION
//  Set BOARD_TYPE to match the physical hardware being flashed.
// -----------------------------------------------------------------------------
#define BOARD_WAVESHARE_S3     1
#define BOARD_MAKERFABS_A7670  2

#define BOARD_TYPE  BOARD_MAKERFABS_A7670   // change to BOARD_WAVESHARE_S3 for Waveshare board

#if   BOARD_TYPE == BOARD_WAVESHARE_S3
  #include "boards/waveshare_s3.h"
#elif BOARD_TYPE == BOARD_MAKERFABS_A7670
  #include "boards/makerfabs_a7670.h"
#else
  #error "Unknown BOARD_TYPE — add a matching entry in board_config.h"
#endif

// -----------------------------------------------------------------------------
//  BOARD IDENTITY  (unique per physical board — change for every new unit)
// -----------------------------------------------------------------------------

// MQTT client ID — must be unique across all boards on the broker.
// Convention: ourwater_<3-digit-number>
#define MQTT_CLIENT   "ourwater_001"

// MQTT topic root — ties this board to a specific site + meter row in Supabase.
// Format: ourwater/<organisation>/<site_identifier>
// Must exactly match the BASE_TOPIC used when creating the meters table entry.
#define BASE_TOPIC    "ourwater/ourwater_botswana/site_001_meter_01"

// -----------------------------------------------------------------------------
//  DEPLOYMENT MODE
// -----------------------------------------------------------------------------

// TEST_MODE true  → publishes every 1 minute regardless of configured interval.
//           false → uses the interval stored in flash/Supabase (default 30 min).
// Always set false before field deployment.
#define TEST_MODE     true

// -----------------------------------------------------------------------------
//  SHARED CREDENTIALS  (same for all boards — do not change)
// -----------------------------------------------------------------------------

#define MQTT_BROKER   "tcp://g7214f51.ala.eu-central-1.emqxsl.com:8883"
#define MQTT_USER     "ourwater_esp32"
#define MQTT_PASS     "@1Mandela1234"
