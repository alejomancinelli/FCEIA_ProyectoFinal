#include <Arduino.h>
#include "BL0940_MQTT.hpp"

#define BL0940_DEBUG 1
#if BL0940_DEBUG
#define DBG(...) \
  { Serial.println(__VA_ARGS__); }
#define ERR(...) \
  { Serial.println(__VA_ARGS__); }
#else
#define DBG(...)
#define ERR(...)
#endif /* BL0940_DBG */

void BL0940PubClient::connectToMQTTBroker()
{
  while (!connected()) {
    String client_id = "esp8266-client-" + String(WiFi.macAddress()); // TODO: Poner otro ID capaz? Aunque lo de la MAC está bueno
    char message[128];
    sprintf(message, "Connecting to MQTT Broker as %s.....", client_id.c_str());
    DBG(message);
    
    if (connect(client_id.c_str())) {
      DBG("Connected to MQTT broker");
      // Publish message upon successful connection
      publish("status", "Hi EMQX I'm ESP8266 ^^"); // TODO: Cambiar esto
    } else {
      char errMessage[128];
      sprintf(errMessage, "Failed to connect to MQTT broker, rc=%d. Try again in 5 seconds", state());
      ERR(errMessage);
      delay(5000); // TODO: Esto capaz sacarlo? Aunque si no puede publicar es lo mismo que nada
    }
  }
}

void BL0940PubClient::publish_(const char* topic, float value)
{
  char buffer[16];

  // float to char*
  snprintf(buffer, sizeof(buffer), "%f", value);
  publish(topic, buffer);
}

void BL0940PubClient::publishVoltageRMS(float voltageRMS)
{
  publish_(VOLTAGE_RMS_TOPIC, voltageRMS);
}

void BL0940PubClient::publishCurrentRMS(float currentRMS)
{
  publish_(CURRENT_RMS_TOPIC, currentRMS);
}

void BL0940PubClient::publishActivePower(float activePower)
{
  publish_(ACTIVE_POWER_TOPIC, activePower);
}

void BL0940PubClient::publishActiveEnergy(float activeEnergy)
{
  publish_(ACTIVE_ENERGY_TOPIC, activeEnergy);
}

void BL0940PubClient::publishPowerFactor(float powerFactor)
{
  publish_(POWER_FACTOR_TOPIC, powerFactor);
}

void BL0940PubClient::publishTemperature(float temperaute)
{
  publish_(TEMPERATURE_TOPIC, temperaute);
}
