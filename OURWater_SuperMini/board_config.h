// =============================================================================
//  OURWater Super Mini — Per-site configuration
//  Edit ONLY this file when flashing a new board.
// =============================================================================

#define WIFI_SSID      "HUAWEI_H112_F77D"
#define WIFI_PASS      "0MNTGGGQJGQ"

// MQTT client ID — must be unique across all boards on the broker.
#define MQTT_CLIENT    "ourwater_sm_001"

// MQTT topic root — must exactly match the base_topic in the meters table.
#define BASE_TOPIC     "ourwater/jwaneng/jwana_primary"

// Serial number — must exactly match serial_no in the meters table.
#define SERIAL_NO      "OW-SM-001"

// TEST_MODE true  → publish every 1 min, dongle cycle every 5 min.
// TEST_MODE false → publish every 30 min, dongle cycle per Supabase settings.
#define TEST_MODE      false

// Shared MQTT broker credentials — same for all boards, do not change.
#define MQTT_BROKER    "g7214f51.ala.eu-central-1.emqxsl.com"
#define MQTT_PORT      8883
#define MQTT_USER      "ourwater_esp32"
#define MQTT_PASS      "@1Mandela1234"

// Supabase — used on startup to fetch dongle timing settings.
// Use the service role key (or anon key if RLS is configured).
#define SUPABASE_URL   "https://jqctwxqbkxdtocqpbfgf.supabase.co"
#define SUPABASE_KEY   "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImpxY3R3eHFia3hkdG9jcXBiZmdmIiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTc3Nzg5NTA2OSwiZXhwIjoyMDkzNDcxMDY5fQ.DqYZw8UJwljHHCtCBzhL9TOosj3xqbDeKbTmFt_AxAo"
