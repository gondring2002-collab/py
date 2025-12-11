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

const char* ssid = "SalsaModem";
const char* password = "Salsa3349@";

const char* mqttServer = "f4159c80eb524405b694dc0e6a311fe4.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUser = "hivemq.webclient.1765435980051";
const char* mqttPassword = "2:E?Q;RJ08&1cijeFbaT";
const char* mqttClientId = "ESP32_A_Airmon";

const char* SENSOR_TOPIC = "sensor/airquality/A";
const char* OTA_TOPIC    = "esp32/A/ota";
const char* OTA_STATUS_TOPIC = "esp32/A/ota/status";
const char* OTA_SECRET = "MY_SECRET_TOKEN_A";

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
#de
