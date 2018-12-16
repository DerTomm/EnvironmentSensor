#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiMan.h>
#include <dht.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_TSL2561_U.h>

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

// TSL2561 parameters
#define INTERVAL_TSL2561 1000
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// Sound sensor parameters
#define INTERVAL_SOUND 1000
#define PIN_SOUND A0

// MQTT parameters
#define MQTT_MESSAGE_INTERVAL 30000
#define MQTT_TOPIC_OUT "deviceNet/out"
#define MQTT_TOPIC_IN "deviceNet/in"
#define MQTT_TOPIC_REGISTRY "deviceNet/registry"
#define MQTT_TOPIC_STATUS "deviceNet/status"
#define DEVICE_NAME "EnvironmentSensor"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// General variables
unsigned long lastDhtTimestamp = 0UL;
unsigned long lastSgpTimestamp = 0UL;
unsigned long lastTslTimestamp = 0UL;
unsigned long lastSoundTimestamp = 0UL;
unsigned long lastMqttPublishTimestamp = 0UL;

#define VALUE_ID_TEMPERATURE 0
#define VALUE_ID_HUMIDITY 1
#define VALUE_ID_LIGHT_LEVEL 2
#define VALUE_ID_NOISE_LEVEL 3
#define VALUE_ID_TVOC 4
#define VALUE_ID_CO2 5

/******************************************************************************
 * 
 */
void reconnectToBroker() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //mqttClient.publish("outTopic", "hello world");
      // ... and resubscribe
      mqttClient.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/******************************************************************************
 * 
 */
void publishValue(int applicationId, int messageType, String contentString) {
  String topic = String(MQTT_TOPIC_OUT) + "/" + String(DEVICE_NAME) + "/" + applicationId + "/1/" + messageType;  // command type = 1/set
  Serial.print("publish: ");
  Serial.print(topic);
  Serial.print(" | ");
  Serial.println(contentString);
  mqttClient.publish(topic.c_str(), contentString.c_str());
}

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

  if (!sgp.begin()) {
    Serial.println("Error: Could not initialize SGP30 sensor.");
    while (1) {
    }
  }

  if (!tsl.begin()) {
    Serial.println("Error: Could not initialize TSL2561 sensor.");
    while (1) {
    }
  }

  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);

  String mqttBrokerAddress = "192.168.64.253";
  mqttClient.setServer(mqttBrokerAddress.c_str(), 1883);
  //mqttClient.setCallback(callback);
}

/******************************************************************************
 * 
 */
void loop() {

  if (!mqttClient.connected()) {
    reconnectToBroker();
  }
  mqttClient.loop();

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
  if (millis() - lastTslTimestamp >= INTERVAL_TSL2561 || millis() - lastTslTimestamp < 0) {
    sensors_event_t event;
    tsl.getEvent(&event);

    /* Display the results (light is measured in lux) */
    if (event.light) {
      Serial.print(event.light);
      Serial.println(" lux");
    } else {
      /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
      Serial.println("Sensor overload");
    }
    lastTslTimestamp = millis();
  }

  // Sound Sensor V2 Noise Level
  if (millis() - lastSoundTimestamp >= INTERVAL_SOUND || millis() - lastSoundTimestamp < 0) {
    int noiseLevel = analogRead(PIN_SOUND);
    Serial.print("Noise: ");
    Serial.println(noiseLevel);
    lastSoundTimestamp = millis();
  }

  // MQTT handling
  //myNode.publishValue(CHILD_ID_TEMPERATURE, V_TEMP, tempString);
  if (millis() - lastMqttPublishTimestamp >= MQTT_MESSAGE_INTERVAL || millis() - lastMqttPublishTimestamp < 0) {

    if (temperature < 85 && temperature > -30) {
      String tempString(temperature, 1);
      publishValue(VALUE_ID_TEMPERATURE, 0, tempString);
    }

    lastMqttPublishTimestamp = millis();
  }
}