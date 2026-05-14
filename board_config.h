// =============================================================================
//  OURWater Board Configuration
//  Edit ONLY this file when flashing a new board.
//  Everything else (OURWater.ino) stays the same.
// =============================================================================

// -----------------------------------------------------------------------------
//  BOARD IDENTITY  (must be unique per physical board)
// -----------------------------------------------------------------------------

// MQTT client ID — every board on the broker must have a different ID.
// Convention: ourwater_<3-digit-number>
#define MQTT_CLIENT   "ourwater_001"

// MQTT topic root — ties this board to a specific site + meter row in Supabase.
// Format: ourwater/<organisation>/<site_id>_<meter_id>
// Must exactly match the entry you create in the meters table.
#define BASE_TOPIC    "ourwater/ourwater_botswana/site_001_meter_01"

// -----------------------------------------------------------------------------
//  DEPLOYMENT MODE
// -----------------------------------------------------------------------------

// TEST_MODE true  → publishes every 1 minute regardless of configured interval.
//           false → uses the interval stored in Supabase (default 30 min).
// Always set false before field deployment.
#define TEST_MODE     true

// -----------------------------------------------------------------------------
//  SHARED CREDENTIALS  (same for all boards — do not change)
// -----------------------------------------------------------------------------

#define MQTT_BROKER   "tcp://g7214f51.ala.eu-central-1.emqxsl.com:8883"
#define MQTT_USER     "ourwater_esp32"
#define MQTT_PASS     "@1Mandela1234"
