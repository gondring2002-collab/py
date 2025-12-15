#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_AirQualitySensor.h"
#include "DFRobot_MultiGasSensor.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

// ---------- CONFIG ----------
#define FIRMWARE_VERSION "1.0.0"

const char* ssid = "Brin-Net";
const char* password = "";

const char* mqttServer = "f4159c80eb524405b694dc0e6a311fe4.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUser = "hivemq.webclient.1765435980051";
const char* mqttPassword = "2:E?Q;RJ08&1cijeFbaT";
const char* mqttClientId = "ESP32_C_Airmon";

const char* SENSOR_TOPIC = "sensor/airquality/C";
const char* OTA_TOPIC    = "esp32/C/ota";
const char* OTA_STATUS_TOPIC = "esp32/C/ota/status";
const char* OTA_SECRET = "MY_SECRET_TOKEN";

// ---------- I2C ADDRESSES & PINS ----------
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

// CO2 PWM
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
    if (flag==2){ flag=4; if (pwmHighStartTicks>pwmHighEndTicks) pwmLowVal=pwmHighStartTicks-pwmHighEndTicks; }
    else flag=1;
  } else {
    pwmHighEndTicks=micros();
    if (flag==1){ flag=2; if (pwmHighEndTicks>pwmHighStartTicks) pwmHighVal=pwmHighEndTicks-pwmHighStartTicks; }
  }
}

float getCO2ppm() {
  if (flag==4){
    flag=1;
    float pwmHighVal_ms = (pwmHighVal*1000.0)/(pwmLowVal+pwmHighVal);
    if (pwmHighVal_ms>=80.0 && pwmHighVal_ms<998.0){
      co2Buffer[co2Index++%AVG_COUNT] = (pwmHighVal_ms-2)*5;
      float sum=0; for(int i=0;i<AVG_COUNT;i++) sum+=co2Buffer[i];
      lastCO2ppm=sum/AVG_COUNT;
    }
  }
  return lastCO2ppm;
}

// ---------- Networking & MQTT ----------
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 5000UL; // 5s

// ---------- Helpers ----------
void publishJson(JsonDocument &doc, const char* topic) {
  char buf[512];
  size_t n = serializeJson(doc, buf);
  mqttClient.publish(topic, buf, n);
}

void publishOtaStatus(const char* status){
  StaticJsonDocument<200> d;
  d["status"]=status;
  d["version"]=FIRMWARE_VERSION;
  char b[200]; size_t n=serializeJson(d,b);
  mqttClient.publish(OTA_STATUS_TOPIC,b,n);
}

// ---------- OTA ----------
void performOta(const char* url){
  publishOtaStatus("starting");
  t_httpUpdate_return ret = httpUpdate.update(wifiClient, url);
  switch(ret){
    case HTTP_UPDATE_FAILED: publishOtaStatus("failed"); break;
    case HTTP_UPDATE_NO_UPDATES: publishOtaStatus("no_updates"); break;
    case HTTP_UPDATE_OK: publishOtaStatus("ok_reboot"); break;
  }
}

// ---------- MQTT callback ----------
void mqttCallback(char* topic, byte* payload, unsigned int length){
  String sPayload;
  for(unsigned int i=0;i<length;i++) sPayload+=(char)payload[i];
  if(String(topic)==OTA_TOPIC){
    StaticJsonDocument<512> jd;
    if(deserializeJson(jd,sPayload)) { publishOtaStatus("invalid_json"); return; }
    if(String(jd["cmd"])=="update" && jd["url"]){
      if(String(jd["token"])!=OTA_SECRET){ publishOtaStatus("invalid_token"); return; }
      performOta(jd["url"]);
    }
  }
}

// ---------- SETUP ----------
void setup(){
  Serial.begin(115200);
  Wire.begin();
  bme.begin(BME280_ADDR);
  ads.begin(ADS1115_ADDR); ads.setGain(GAIN_ONE);
  pm25.begin();
  no2.begin(); so2.begin();
  pinMode(SENSOR_DATA_PIN, INPUT);
  attachInterrupt(INTERRUPT_NUMBER, interruptChange, CHANGE);

  WiFi.begin(ssid,password);
  Serial.print("WiFi connecting...");
  while(WiFi.status()!=WL_CONNECTED){ Serial.print("."); delay(500); }
  Serial.println(WiFi.localIP());

  wifiClient.setInsecure(); // HiveMQ TLS tanpa sertifikat
  mqttClient.setServer(mqttServer,mqttPort);
  mqttClient.setCallback(mqttCallback);

  Serial.println("Setup done");
  publishOtaStatus("boot");
}

// ---------- LOOP ----------
void loop(){
  if(!mqttClient.connected()){
    Serial.print("MQTT connecting...");
    if(mqttClient.connect(mqttClientId,mqttUser,mqttPassword)){
      Serial.println("connected");
      mqttClient.subscribe(OTA_TOPIC);
      publishOtaStatus("mqtt_connected");
    } else { Serial.print("failed rc="); Serial.println(mqttClient.state()); delay(3000); return; }
  }
  mqttClient.loop();

  unsigned long now=millis();
  if(now-lastPublish>=PUBLISH_INTERVAL){
    lastPublish=now;

    StaticJsonDocument<512> doc;
    doc["dev"]="C"; doc["version"]=FIRMWARE_VERSION;
    doc["temp"]=bme.readTemperature();
    doc["hum"]=bme.readHumidity();
    doc["press"]=bme.readPressure()/100.0;
    doc["ch0"]=ads.readADC_SingleEnded(0)*0.1875/1000.0;
    doc["ch1"]=ads.readADC_SingleEnded(1)*0.1875/1000.0;
    doc["co2"]=getCO2ppm();
    doc["pm25"]=pm25.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD);
    doc["pm10"]=pm25.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD);
    doc["no2"]=no2.readGasConcentrationPPM();
    doc["so2"]=so2.readGasConcentrationPPM();

    publishJson(doc,SENSOR_TOPIC);
    Serial.println("Published sensor JSON");
  }
  delay(10);
}
