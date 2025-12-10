#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_AirQualitySensor.h"
#include "DFRobot_MultiGasSensor.h"
#include <ArduinoJson.h>

// ---------- CONFIG ----------
#define FIRMWARE_VERSION "1.0.0"

const char* ssid = "SalsaModem";
const char* password = "Salsa3349@";

const char* mqttServer = "test.mosquitto.org";
const int   mqttPort   = 1883;
const char* mqttClientId = "ESP32_C_Airmon";

const char* SENSOR_TOPIC = "sensor/airquality/C";
const char* OTA_TOPIC    = "esp32/C/ota";
const char* OTA_STATUS_TOPIC = "esp32/C/ota/status";
const char* OTA_SECRET = "MY_SECRET_TOKEN";

// ---------- I2C ADDRESSES & Pins ----------
#define BME280_ADDR   0x76
#define ADS1115_ADDR  0x48
#define PM25_ADDR     0x19
#define NO2_ADDR      0x74
#define SO2_ADDR      0x75

#define SENSOR_DATA_PIN   32
#define INTERRUPT_NUMBER  digitalPinToInterrupt(SENSOR_DATA_PIN)

// ---------- Objects ----------
Adafruit_BME280 bme;
Adafruit_ADS1115 ads;
DFRobot_AirQualitySensor pm25(&Wire, PM25_ADDR);
DFRobot_GAS_I2C no2(&Wire, NO2_ADDR);
DFRobot_GAS_I2C so2(&Wire, SO2_ADDR);

// CO2 PWM variables
volatile unsigned long pwmHighStartTicks=0, pwmHighEndTicks=0;
volatile unsigned long pwmHighVal=0, pwmLowVal=0;
volatile uint8_t flag=0;

#define AVG_COUNT 5
float co2Buffer[AVG_COUNT] = {0};
uint8_t co2Index = 0;
float lastCO2ppm = -1;

void IRAM_ATTR interruptChange() {
  if (digitalRead(SENSOR_DATA_PIN)) {
    pwmHighStartTicks = micros();
    if (2 == flag) {
      flag = 4;
      if (pwmHighStartTicks > pwmHighEndTicks) pwmLowVal = pwmHighStartTicks - pwmHighEndTicks;
    } else flag = 1;
  } else {
    pwmHighEndTicks = micros();
    if (1 == flag) {
      flag = 2;
      if (pwmHighEndTicks > pwmHighStartTicks) pwmHighVal = pwmHighEndTicks - pwmHighStartTicks;
    }
  }
}

float getCO2ppm() {
  if (flag == 4) {
    flag = 1;
    float pwmHighVal_ms = (pwmHighVal * 1000.0) / (pwmLowVal + pwmHighVal);
    if (pwmHighVal_ms >= 80.0 && pwmHighVal_ms < 998.0) {
      float ppm = (pwmHighVal_ms - 2) * 5;
      co2Buffer[co2Index++ % AVG_COUNT] = ppm;
      float sum = 0;
      for (int i = 0; i < AVG_COUNT; i++) sum += co2Buffer[i];
      lastCO2ppm = sum / AVG_COUNT;
    }
  }
  return lastCO2ppm;
}

// ---------- Networking & MQTT ----------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 5000UL; // 5s

// ---------- Helpers ----------
void publishJson(JsonDocument &doc, const char* topic) {
  char buf[512];
  size_t n = serializeJson(doc, buf);
  mqttClient.publish(topic, buf, n);
}

void publishOtaStatus(const char* status) {
  StaticJsonDocument<200> d;
  d["status"] = status;
  d["version"] = FIRMWARE_VERSION;
  char b[200];
  size_t n = serializeJson(d, b);
  mqttClient.publish(OTA_STATUS_TOPIC, b, n);
}

// ---------- OTA ----------
void performOta(const char* url) {
  Serial.printf("Starting OTA from: %s\n", url);
  publishOtaStatus("starting");
  t_httpUpdate_return ret = httpUpdate.update(wifiClient, url);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      {
        int err = httpUpdate.getLastError();
        String msg = "HTTP_UPDATE_FAILED: " + String(err) + " " + String(httpUpdate.getLastErrorString());
        Serial.println(msg);
        publishOtaStatus(msg.c_str());
      }
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      publishOtaStatus("no_updates");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK - rebooting");
      publishOtaStatus("ok_reboot");
      break;
  }
}

// ---------- MQTT callback ----------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String sTopic = String(topic);
  String sPayload;
  sPayload.reserve(length+1);
  for (unsigned int i=0;i<length;i++) sPayload += (char)payload[i];
  Serial.printf("MQTT RX [%s] : %s\n", topic, sPayload.c_str());

  if (sTopic == OTA_TOPIC) {
    StaticJsonDocument<512> jd;
    DeserializationError err = deserializeJson(jd, sPayload);
    if (err) {
      Serial.println("OTA payload parse error");
      publishOtaStatus("invalid_json");
      return;
    }
    const char* cmd = jd["cmd"];
    const char* url = jd["url"];
    const char* token = jd["token"];
    const char* version = jd["version"];

    if (!cmd) { publishOtaStatus("missing_cmd"); return; }
    if (String(cmd) == "update") {
      if (strlen(OTA_SECRET) > 0) {
        if (!token || String(token) != String(OTA_SECRET)) {
          publishOtaStatus("invalid_token");
          Serial.println("OTA attempt with invalid token");
          return;
        }
      }
      if (version) {
        String newV = String(version);
        if (newV == String(FIRMWARE_VERSION)) {
          publishOtaStatus("same_version");
          return;
        }
      }
      if (url) {
        publishOtaStatus("starting_update");
        performOta(url);
      } else publishOtaStatus("missing_url");
    } else publishOtaStatus("unknown_cmd");
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Sensors
  if (!bme.begin(BME280_ADDR)) Serial.println("BME280 not found!");
  if (!ads.begin(ADS1115_ADDR)) Serial.println("ADS1115 not found!"); else ads.setGain(GAIN_ONE);

  while(!pm25.begin()){ Serial.println("PM2.5 init fail"); delay(1000); }
  while(!no2.begin()){ no2.changeAcquireMode(no2.PASSIVITY); no2.setTempCompensation(no2.OFF); delay(1000); }
  while(!so2.begin()){ so2.changeAcquireMode(so2.PASSIVITY); so2.setTempCompensation(so2.OFF); delay(1000); }

  pinMode(SENSOR_DATA_PIN, INPUT);
  attachInterrupt(INTERRUPT_NUMBER, interruptChange, CHANGE);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting...");
  unsigned long tstart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - tstart < 20000) { Serial.print("."); delay(500); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) Serial.println(WiFi.localIP());

  // MQTT
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);

  Serial.println("Setup done.");
  publishOtaStatus("boot");
}

// ---------- LOOP ----------
void loop() {
  if (!mqttClient.connected()) {
    Serial.print("MQTT connecting...");
    if (mqttClient.connect(mqttClientId)) {
      Serial.println("connected");
      mqttClient.subscribe(OTA_TOPIC);
      publishOtaStatus("mqtt_connected");
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
      delay(3000);
      return;
    }
  }
  mqttClient.loop();

  // Publish sensor periodically
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;

    float temp = bme.readTemperature();
    float hum  = bme.readHumidity();
    float press= bme.readPressure() / 100.0;
    float ch0 = ads.readADC_SingleEnded(0) * 0.1875 / 1000.0;
    float ch1 = ads.readADC_SingleEnded(1) * 0.1875 / 1000.0;
    float co2ppm = getCO2ppm();
    uint16_t pm25Val = pm25.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD);
    uint16_t pm10Val = pm25.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD);
    float no2ppm = no2.readGasConcentrationPPM();
    float so2ppm = so2.readGasConcentrationPPM();

    StaticJsonDocument<512> doc;
    doc["dev"] = "C";
    doc["version"] = FIRMWARE_VERSION;
    doc["temp"] = temp;
    doc["hum"]  = hum;
    doc["press"]= press;
    doc["co2"]  = co2ppm > 0 ? co2ppm : -1;
    doc["pm25"] = pm25Val;
    doc["pm10"] = pm10Val;
    doc["no2"]  = no2ppm;
    doc["so2"]  = so2ppm;
    doc["ch0"]  = ch0;
    doc["ch1"]  = ch1;

    publishJson(doc, SENSOR_TOPIC);
    Serial.println("Published sensor JSON");
  }

  delay(10);
}
