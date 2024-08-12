#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "BL0940_MQTT.hpp"

// WiFi settings
const char *ssid = "Capitan";               // Replace with your WiFi name
const char *password = "EntreRios1355";     // Replace with your WiFi password

// MQTT Broker settings
const char *mqtt_broker = "192.168.1.22"; // EMQX broker endpoint
const char *mqtt_topic = "emqx/esp8266";    // MQTT topic
const int mqtt_port = 1883;                 // MQTT port (TCP)

WiFiClient espClient;
BL0940PubClient mqtt_client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
float value = 0;

void connectToWiFi();

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  mqtt_client.setServer(mqtt_client.getMqttBroker(), mqtt_client.getMqttPort());
  mqtt_client.connectToMQTTBroker();

  // TODO: El keepalive es importante !!!
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to the WiFi network");
}

void loop() {
  if (!mqtt_client.connected()) {
    mqtt_client.connectToMQTTBroker();
  }
  mqtt_client.loop();

  // Publish random Voltage RMS
  long randVoltage_l = random(21000, 23000);
  float randVoltage_f = float(randVoltage_l) / 100.0;
  mqtt_client.publishVoltageRMS(randVoltage_f);
  
  // Publish random Current RMS
  long randCurrent_l = random(1, 30000);
  float randCurrent_f = float(randCurrent_l) / 100.0;
  mqtt_client.publishCurrentRMS(randCurrent_f);
  
  // Publish random Active Power
  long randActivePower_l = random(1, 1000000);
  float randActivePower_f = float(randActivePower_l) / 100.0;
  mqtt_client.publishActivePower(randActivePower_f);

  // Publish random Active Energy
  long randActiveEnergy_l = random(1, 1000000);
  float randActiveEnergy_f = float(randActiveEnergy_l) / 100.0;
  mqtt_client.publishActiveEnergy(randActiveEnergy_f);
  
  // Publish random Power Factor
  long randActivePowerFactor_l = random(1, 1000);
  float randActivePowerFactor_f = float(randActivePowerFactor_l) / 1000.0;
  mqtt_client.publishPowerFactor(randActivePowerFactor_f);
  
  // Publish random Temperature
  long randActiveTemperature_l = random(2000, 8000);
  float randActiveTemperature_f = float(randActiveTemperature_l) / 100.0;
  mqtt_client.publishTemperature(randActiveTemperature_f);
  
  delay(500);

  // if (now - lastMsg > 2000) {
  //   lastMsg = now;
  //   ++value;
    
  //   Serial.print("Publish message: ");
  //   Serial.println(msg);
  //   mqtt_client.publishVoltageRMS(value);
  // }
}
