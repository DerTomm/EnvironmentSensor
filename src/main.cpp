#include <Arduino.h>
#include <WiFiMan.h>
#include <dht.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"

// WifiMan parameters
Config wifiConfig;

// DHT22 parameters
#define INTERVAL_DHT22 60000
#define DHT22_PIN D3
dht DHT;
float humidity;
float temperature;

// SGP30 parameters
#define INTERVAL_SGP30 30000
Adafruit_SGP30 sgp;
int sgpCounter = 0;

// General variables
unsigned long lastDhtTimestamp = 0UL;
unsigned long lastSgpTimestamp = 0UL;

/******************************************************************************
 * 
 */
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));  // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                 // [mg/m^3]
  return absoluteHumidityScaled;
}

/******************************************************************************
 * 
 */
void setup() {

  Serial.begin(115200);

  //create default object
  WiFiMan wifiMan = WiFiMan();
  wifiMan.start();

  if (wifiMan.getConfig(&wifiConfig)) {
    //display device status
    Serial.println("Connected to AP");
    Serial.print("SSID : ");
    Serial.println(wifiConfig.wifiSsid);
    Serial.print("Passwd : ");
    Serial.println(wifiConfig.wifiPasswd);
    Serial.print("MQTT server : ");
    Serial.println(wifiConfig.mqttAddr);
    Serial.print("MQTT password : ");
    Serial.println(wifiConfig.mqttPasswd);
    Serial.print("MQTT username : ");
    Serial.println(wifiConfig.mqttUsername);
    Serial.print("MQTT Id : ");
    Serial.println(wifiConfig.mqttId);
    Serial.print("Sub topic : ");
    Serial.println(wifiConfig.mqttSub);
    Serial.print("Pub topic : ");
    Serial.println(wifiConfig.mqttPub);
    Serial.print("MQTT port : ");
    Serial.println(wifiConfig.mqttPort);
    Serial.print("Master password : ");
    Serial.println(wifiConfig.masterPasswd);
    Serial.print("Device mDNS name : ");
    Serial.println(wifiConfig.mdnsName);
    Serial.print("IP : ");
    Serial.println(wifiConfig.localIP);
  }

  sgp.begin();
}

/******************************************************************************
 * 
 */
void loop() {

  // DHT22 temperature and humidity reading
  if (millis() - lastDhtTimestamp >= INTERVAL_DHT22 || millis() - lastDhtTimestamp < 0) {
    int chk = DHT.read22(DHT22_PIN);
    if (chk == DHTLIB_OK) {
      temperature = DHT.temperature;
      humidity = DHT.humidity;
      Serial.print("DHT22: ");
      Serial.print(temperature);
      Serial.print(" | ");
      Serial.println(humidity);
    } else {
      Serial.println("DHT22 error.");
      temperature = humidity = -255.0;
    }
    lastDhtTimestamp = millis();
  }

  // SGP30 Gas Sensor
  if (millis() - lastSgpTimestamp >= INTERVAL_SGP30 || millis() - lastSgpTimestamp < 0) {
    if (temperature > -255.0 && humidity > -255.0) {
      sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));
    }
    if (sgp.IAQmeasure()) {
      Serial.print("TVOC ");
      Serial.print(sgp.TVOC);
      Serial.print(" ppb\t");
      Serial.print("eCO2 ");
      Serial.print(sgp.eCO2);
      Serial.println(" ppm");
      sgpCounter++;
      if (sgpCounter == 30) {
        sgpCounter = 0;
        uint16_t TVOC_base, eCO2_base;
        if (!sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
          Serial.print("****Baseline values: eCO2: 0x");
          Serial.print(eCO2_base, HEX);
          Serial.print(" & TVOC: 0x");
          Serial.println(TVOC_base, HEX);
        }
      }
    }
    lastSgpTimestamp = millis();
  }

  // TSL2561 Light Sensor

  // Sound Sensor V2 Noise Level
}