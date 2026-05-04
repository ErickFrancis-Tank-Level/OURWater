HardwareSerial modemSerial(1);

#define MODEM_POWER 33
#define MODEM_RX    17
#define MODEM_TX    18
#define PIN_REED    21
#define PIN_RELAY   19

#define PULSES_PER_LITRE    10
#define PUBLISH_INTERVAL_MS 60000

volatile uint32_t pulseCount    = 0;
volatile uint32_t lastPulseTime = 0;
bool     valveOpen     = false;
uint32_t lastPublishMs = 0;
uint32_t totalLitres   = 0;
bool     mqttConnected = false;

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

void IRAM_ATTR onPulse() {
  uint32_t now = millis();
  if (now - lastPulseTime > 50) {
    pulseCount++;
    lastPulseTime = now;
  }
}

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

void publishMessage(const char* topic, const char* payload);

void modemPowerOn() {
  Serial.println("[Modem] Powering on...");
  pinMode(MODEM_POWER, OUTPUT);
  digitalWrite(MODEM_POWER, HIGH);
  for (int i = 0; i < 80; i++) { delay(100); yield(); }
  modemSerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1000);
  Serial.println("[Modem] Power on complete");
}

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

bool networkInit() {
  Serial.print("[Modem] Waiting for AT");
  for (int i = 0; i < 30; i++) {
    String r = sendAT("AT", "OK", 1000);
    if (r.indexOf("OK") >= 0) { Serial.println(" OK"); break; }
    if (i == 29) { Serial.println(" FAILED"); return false; }
    Serial.print(".");
    delay(500);
  }
  sendAT("ATE0", "OK", 2000);
  sendAT("AT+CMEE=2", "OK", 2000);
  Serial.print("[Network] Registering");
  for (int i = 0; i < 30; i++) {
    String r = sendAT("AT+CREG?", "OK", 2000);
    if (r.indexOf(",1") >= 0 || r.indexOf(",5") >= 0) { Serial.println(" OK"); break; }
    if (i == 29) { Serial.println(" FAILED"); return false; }
    Serial.print(".");
    delay(2000);
  }
  sendAT("AT+CSQ", "OK", 2000);
  sendAT("AT+COPS?", "OK", 2000);
  sendAT("AT+NETCLOSE", "+NETCLOSE", 15000);  // force-close stale context so DNS resets
  delay(2000);
  String r = sendAT("AT+NETOPEN", "+NETOPEN: 0", 15000);
  if (r.indexOf("+NETOPEN: 0") < 0) { Serial.println("[Network] NETOPEN failed"); return false; }
  sendAT("AT+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"", "OK", 3000);  // override carrier DNS
  r = sendAT("AT+IPADDR", "+IPADDR", 5000);
  if (r.indexOf("+IPADDR") < 0) { Serial.println("[Network] No IP"); return false; }

  Serial.println("[DNS] Resolving broker...");
  sendAT("AT+CDNSGIP=\"g7214f51.ala.eu-central-1.emqxsl.com\"", "+CDNSGIP", 10000);

  Serial.println("[NTP] Syncing...");
  sendAT("AT+CNTP=\"pool.ntp.org\",0", "OK", 5000);
  sendAT("AT+CNTP", "+CNTP", 10000);
  sendAT("AT+CCLK?", "OK", 2000);

  loadCACert();

  sendAT("AT+CSSLCFG=\"sslversion\",0,4", "OK", 2000);
  sendAT("AT+CSSLCFG=\"authmode\",0,1", "OK", 2000);
  sendAT("AT+CSSLCFG=\"ignorelocaltime\",0,1", "OK", 2000);
  sendAT("AT+CSSLCFG=\"cacert\",0,\"emqx-ca.pem\"", "OK", 2000);
  sendAT("AT+CSSLCFG=\"enableSNI\",0,1", "OK", 2000);

  Serial.println("[Network] Ready");
  return true;
}

bool mqttConnect() {
  Serial.println("[Net] Checking internet connectivity...");
  String dns = sendAT("AT+CDNSGIP=\"google.com\"", "+CDNSGIP", 10000);
  if (dns.indexOf("+CDNSGIP: 1") < 0) {
    Serial.println("[Net] DNS failed — no internet, skipping MQTT");
    return false;
  }
  Serial.println("[Net] Internet OK");

  Serial.println("[MQTT] Starting...");
  sendAT("AT+CMQTTDISC=0,120", "OK", 15000);  // disconnect broker before releasing
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
    sendAT("AT+CMQTTSTOP", "+CMQTTSTOP", 8000);  // force stop before retry
    delay(3000);
  }
  if (r.indexOf("+CMQTTSTART: 0") < 0) { Serial.println("[MQTT] Start failed"); return false; }

  r = sendAT("AT+CMQTTACCQ=0,\"ourwater_001\",1", "OK", 5000);
  if (r.indexOf("OK") < 0) { Serial.println("[MQTT] Acquire failed"); return false; }

  sendAT("AT+CMQTTSSLCFG=0,0", "OK", 2000);

  Serial.println("[MQTT] Connecting...");
  r = sendAT(
    "AT+CMQTTCONNECT=0,\"tcp://g7214f51.ala.eu-central-1.emqxsl.com:8883\",60,1,\"ourwater_esp32\",\"@1Mandela1234\"",
    "+CMQTTCONNECT: 0,0", 30000);

  if (r.indexOf("+CMQTTCONNECT: 0,0") >= 0) {
    Serial.println("[MQTT] Connected!");
    mqttConnected = true;
    String subTopic = "ourwater/ourwater_botswana/site_001_meter_01/valve/cmd";
    String subCmd = "AT+CMQTTSUBTOPIC=0," + String(subTopic.length()) + ",1";
    sendATData(subCmd.c_str(), subTopic.c_str(), "OK", 5000);
    sendAT("AT+CMQTTSUB=0", "+CMQTTSUB: 0,0", 5000);
    publishMessage("ourwater/ourwater_botswana/site_001_meter_01/status", "{\"status\":\"online\"}");
    return true;
  }
  Serial.println("[MQTT] Connect failed");
  mqttConnected = false;
  return false;
}

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

void publishFlow(uint32_t litres, float lpm) {
  String payload = "{\"total_litres\":" + String(litres) +
                   ",\"flow_lpm\":" + String(lpm, 1) +
                   ",\"valve_open\":" + (valveOpen ? "true" : "false") + "}";
  publishMessage("ourwater/ourwater_botswana/site_001_meter_01/flow", payload.c_str());
}

void checkIncoming() {
  if (modemSerial.available()) {
    String incoming = modemSerial.readString();
    if (incoming.length() > 0) {
      Serial.println("[Modem] " + incoming);
      if (incoming.indexOf("\"open\"") >= 0) {
        valveOpen = true;
        digitalWrite(PIN_RELAY, HIGH);
        Serial.println("[Valve] OPENED");
        publishMessage("ourwater/ourwater_botswana/site_001_meter_01/valve/state", "{\"valve_open\":true}");
      } else if (incoming.indexOf("\"close\"") >= 0) {
        valveOpen = false;
        digitalWrite(PIN_RELAY, LOW);
        Serial.println("[Valve] CLOSED");
        publishMessage("ourwater/ourwater_botswana/site_001_meter_01/valve/state", "{\"valve_open\":false}");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=============================");
  Serial.println(" OURWater Firmware Starting");
  Serial.println("=============================");
  pinMode(PIN_REED, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  attachInterrupt(digitalPinToInterrupt(PIN_REED), onPulse, FALLING);
  Serial.println("[GPIO] Ready");
  modemPowerOn();
  if (!networkInit()) {
    Serial.println("[Boot] Network failed — restarting");
    for (int i = 0; i < 300; i++) { delay(100); yield(); }
    ESP.restart();
  }
  mqttConnect();
  Serial.println("[Boot] System ready");
}

void loop() {
  if (!mqttConnected) {
    Serial.println("[MQTT] Reconnecting...");
    mqttConnect();
    delay(5000);
    return;
  }
  checkIncoming();
  uint32_t now = millis();
  if (now - lastPublishMs >= PUBLISH_INTERVAL_MS) {
    noInterrupts();
    uint32_t pulses = pulseCount;
    pulseCount = 0;
    interrupts();
    uint32_t litres = pulses / PULSES_PER_LITRE;
    totalLitres += litres;
    float lpm = litres / (PUBLISH_INTERVAL_MS / 60000.0f);
    publishFlow(totalLitres, lpm);
    lastPublishMs = now;
  }
  delay(10);
}