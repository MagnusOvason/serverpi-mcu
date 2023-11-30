/*
Board: ESP32 Dev D1 WROOM

Program to publish data to an MQTT broker as a stringified JSON object.
MCU connects to WiFi and MQTT broker, then publishes data to the broker.

No sensors are connected to the MCU yet, so the values are hardcoded in a json object
*/

// Include libraries
#include <Arduino.h>               // Arduino framework
#include <ArduinoJson.h>           // JSON library
#include <WiFi.h>                  // WiFi library
#include <PubSubClient.h>          // MQTT library
#include <string.h>                // For creating strings
#include "time.h"                  // Time library
#include "soc/soc.h"               // Disable brownout problems
#include "soc/rtc_cntl_reg.h"      // Disable brownout problems
#include "I2CSoilMoistureSensor.h" // Library for getting data from the Catnip electronics soil moisture sensor
#include <Wire.h>                  // I2C library
#include "DHT.h"                   // Library for getting data from the DHT22 sensor

// Macro definitions
#define SDA_PIN 21        // SDA
#define SCL_PIN 22        // SCL
#define DHTVCC 16         // DHT22 sensor VCC pin
#define DHTPIN 17         // DHT22 sensor pin
#define DHTTYPE DHT22     // DHT22 sensor type
#define ECHO_PIN 5        // HC-SR04 Echo Pin
#define TRIGGER_PIN 23    // HC-SR04 Trigger Pin
#define LIGHT_PIN 36      // Photoresistor pin
#define LIGHTVCC 26       // Photoresistor VCC pin
#define CATNIP_ADDR 0x23  // Catnip sensor address
#define WATER_PUMP_PIN 19 // Motor pin
#define DELAY_TIME 10000  // Delay time between measurements

// WiFi credentials
const char *ssid = "CG24";
const char *password = "KogtLort23";

// MQTT broker details and credentials
const char *mqttServer = "192.168.2.198";
const int mqttPort = 1883;

// Client user, ID and password
const char *clientId = "mcu3";           // mcu1, mcu2, mcu3
const char *clientUsername = "pub3";     // pub1, pub2, pub3
const char *clientPass = "Letspublish3"; // Letspublish1, Letspublish2, Letspublish3

// Topic to publish to
const char *topic = "topic-sub3"; // topic-sub1, topic-sub2, topic-sub3

// Struct to save measurements
struct Measurements
{
  uint16_t soilMoisture;
  float temperature;
  float humidity;
  float lightIntensity;
  float waterLevel;
};

// Declare objects
WiFiClient espClient;                      // Initialize WiFi client
PubSubClient client(espClient);            // Initialize MQTT client
I2CSoilMoistureSensor sensor(CATNIP_ADDR); // Initialize I2CSoilMoistureSensor sensor (addr: 0x21, 0x22, 0x23)
DHT dht(DHTPIN, DHTTYPE);                  // Initialize DHT sensor
Measurements measurements;

// Variables
const char *ntpServer = "pool.ntp.org"; // NTP server
unsigned long epochTime;                // Variable to save epoch time
unsigned long lastMeasureTime = 0;      // Variable to save last measure time
unsigned long delayTime = DELAY_TIME;   // Delay time between measurements

// function prototypes
void initWiFi();                                                        // Function to initialize WiFi
void connectToMQTT();                                                   // Function to connect to MQTT broker
void disconnectFromMQTT();                                              // Function to disconnect from MQTT broker
unsigned long getTime();                                                // Function to get current epoch time
long map(long x, long in_min, long in_max, long out_min, long out_max); // Function for mapping values
long getLightIntensity();                                               // Function to get light intensity from light sensor
Measurements takeMeasurements();                                        // Function to take measurements
void pinSetup();                                                        // Function to setup pins

void setup()
{
  // disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // setup pins
  pinSetup();

  // initialize I2C
  Wire.begin();

  // initialize serial communication
  Serial.begin(115200);

  // initialize WiFi
  initWiFi();

  // initialize MQTT
  client.setServer(mqttServer, mqttPort);
  client.setBufferSize(512);
  client.setKeepAlive(15);
  client.setSocketTimeout(15);
  connectToMQTT();

  // initialize NTP
  configTime(0, 0, ntpServer); // UTC time

  // initialize soil moisture sensor
  sensor.begin();
  delay(1000); // wait for sensor to initialize
  Serial.print("I2C Soil Moisture Sensor Address: ");
  Serial.println(sensor.getAddress(), HEX);
  Serial.print("Sensor Firmware version: ");
  Serial.println(sensor.getVersion(), HEX);
  Serial.println();

  // initialize DHT22 sensor
  dht.begin();
}

void loop()
{
  // ones every 60 seconds, take measurements and publish to MQTT broker
  if (millis() - lastMeasureTime > delayTime)
  {
    // check WiFi connection
    if (WiFi.status() != WL_CONNECTED)
    {
      initWiFi();
    }

    // check MQTT connection
    if (!client.connected())
    {
      connectToMQTT();
    }

    // take measurements
    measurements = takeMeasurements();
    Serial.println("Measurements taken");
    Serial.println("Soil moisture [Capacitance]: " + String(measurements.soilMoisture));
    Serial.println("Temperature [Celsius]: " + String(measurements.temperature));
    Serial.println("Humidity [Percent]: " + String(measurements.humidity));
    Serial.println("Light intensity [Percent]: " + String(measurements.lightIntensity));
    Serial.println("Water level [Percent]: " + String(measurements.waterLevel));

    // create a json object
    DynamicJsonDocument doc(512);

    // get the current epoch time
    epochTime = getTime();

    doc["timestamp"] = epochTime;

    // add client id
    doc["client_id"] = clientId;

    // add metrics
    JsonArray metrics = doc.createNestedArray("metrics");

    // add soil moisture
    JsonObject soilMoisture = metrics.createNestedObject();
    soilMoisture["name"] = "soil_moisture";
    soilMoisture["dataType"] = "float";
    soilMoisture["value"] = measurements.soilMoisture;
    soilMoisture["unit"] = "percent";

    // add temperature
    JsonObject temperature = metrics.createNestedObject();
    temperature["name"] = "temperature";
    temperature["dataType"] = "float";
    temperature["value"] = measurements.temperature;
    temperature["unit"] = "celsius";

    // add humidity
    JsonObject humidity = metrics.createNestedObject();
    humidity["name"] = "humidity";
    humidity["dataType"] = "float";
    humidity["value"] = measurements.humidity;
    humidity["unit"] = "percent";

    // add light intensity
    JsonObject lightIntensity = metrics.createNestedObject();
    lightIntensity["name"] = "light_intensity";
    lightIntensity["dataType"] = "float";
    lightIntensity["value"] = measurements.lightIntensity;
    lightIntensity["unit"] = "percent";

    // add water level
    JsonObject waterLevel = metrics.createNestedObject();
    waterLevel["name"] = "water_level";
    waterLevel["dataType"] = "float";
    waterLevel["value"] = measurements.waterLevel;
    waterLevel["unit"] = "percent";

    // Stringify the JSON object
    String jsonStr;
    serializeJson(doc, jsonStr);

    // Publish the JSON object
    client.publish(topic, jsonStr.c_str());

    // disconnect from MQTT broker
    disconnectFromMQTT();

    // update last measure time
    lastMeasureTime = millis();
  }
}

// Initialize WiFi
void initWiFi()
{
  delay(10);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to the WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

// Function to connect to MQTT broker
void connectToMQTT()
{
  while (!client.connected())
  {
    // Serial.println("Connecting to MQTT...");
    if (client.connect(clientId, clientUsername, clientPass))
    {
      Serial.println("Connected to MQTT broker");
    }
    else
    {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      Serial.println("Retrying in 5 seconds");
      delay(5000);
    }
  }
}

// Function to disconnect from MQTT broker
void disconnectFromMQTT()
{
  client.disconnect();
  Serial.println("Disconnected from MQTT broker");
}

// Function that gets current epoch time
unsigned long getTime()
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    // Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}

// Function for mapping values
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  float result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

  return result;
}

// Function for getting water level
float getWaterLevel()
{
  // Trigger the sensor by setting it HIGH for 10 microseconds:
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  uint16_t duration = pulseIn(ECHO_PIN, HIGH);
  float distance = ((float)duration * 0.0343) / 2.0;

  return distance;
}

// Function for getting light intensity
long getLightIntensity()
{
  // Turn on the sensor by setting the VCC pin HIGH for 100 microseconds:
  digitalWrite(LIGHTVCC, HIGH);
  delayMicroseconds(100);

  long lightIntensity = analogRead(LIGHT_PIN);

  // map light intensity to a percentage
  lightIntensity = map(lightIntensity, 0, 4095, 0.0, 100.0);

  // Turn the sensor off by setting the VCC pin LOW:
  digitalWrite(LIGHTVCC, LOW);

  return lightIntensity;
}

// Function to take measurements, save them in a struct and return the struct
Measurements takeMeasurements()
{
  // Catnip electronics soil moisture sensor
  uint16_t soilMoisture = sensor.getCapacitance();
  while (sensor.isBusy()) // wait until sensor is ready
  {
    delay(50);
  }
  // discard first measurement - When doing rare measurements and wanting to act instantly, do two consecutive readings to get the most up to date data.
  soilMoisture = sensor.getCapacitance(); // TODO should be uint16_t instead of float

  // Catnip electronics temperature sensor
  uint16_t temperature = sensor.getTemperature();
  while (sensor.isBusy()) // wait until sensor is ready
  {
    delay(50);
  }
  // discard first measurement - When doing rare measurements and wanting to act instantly, do two consecutive readings to get the most up to date data.
  float temperatureFloat = sensor.getTemperature() / 10.0; // divide by 10 to get temperature in degrees celsius

  sensor.sleep();

  // Get humidity
  digitalWrite(DHTVCC, HIGH);
  delay(100); // wait for sensor to initialize
  float humidity = dht.readHumidity();
  digitalWrite(DHTVCC, LOW);

  // Light intensity sensor
  long lightIntensity = getLightIntensity();

  // Water level sensor
  float waterLevel = getWaterLevel();

  // Save measurements in a struct
  Measurements measurements;
  measurements.soilMoisture = (float)soilMoisture;
  measurements.temperature = temperatureFloat;
  measurements.humidity = humidity;
  measurements.lightIntensity = (float)lightIntensity;
  measurements.waterLevel = waterLevel;

  return measurements;
}

// Function to setup pins
void pinSetup()
{
  // Setup pins on the ESP32 Live D1 mini board for the soil moisture sensor (I2C)
  pinMode(SDA_PIN, OUTPUT); // SDA
  pinMode(SCL_PIN, OUTPUT); // SCL

  // Setup pins on the ESP32 Live D1 mini board for the DHT22 sensor
  pinMode(DHTPIN, OUTPUT);
  pinMode(DHTVCC, OUTPUT);
  digitalWrite(DHTVCC, LOW); // initialize DHT22 sensor VCC pin to low

  // Setup pins on the ESP32 Live D1 mini board for the water level sensor
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // initialize trigger pin to low

  // Setup pins on the ESP32 Live D1 mini board for the light intensity sensor
  pinMode(LIGHT_PIN, INPUT);
  pinMode(LIGHTVCC, OUTPUT);
  digitalWrite(LIGHTVCC, LOW); // initialize light sensor VCC pin to low

  // Setup pins on the ESP32 Live D1 mini board for the water pump
  pinMode(WATER_PUMP_PIN, OUTPUT);
  digitalWrite(WATER_PUMP_PIN, LOW); // initialize water pump pin to low
}