#ifndef BL0940_MQTT_hpp
#define BL0940_MQTT_hpp

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

class BL0940PubClient : public PubSubClient 
{
  private: 
    const char *VOLTAGE_RMS_TOPIC =     "BL0940/voltageRMS";
    const char *CURRENT_RMS_TOPIC =     "BL0940/currentRMS";
    const char *ACTIVE_POWER_TOPIC =    "BL0940/activePower";
    const char *ACTIVE_ENERGY_TOPIC =   "BL0940/activeEnergy";
    const char *POWER_FACTOR_TOPIC =    "BL0940/powerFactor";
    const char *TEMPERATURE_TOPIC =     "BL0940/temperatureFactor";

    // TODO: No será constante y se podrá cambiar, al igual que el PORT
    const uint16_t MQTT_PORT = 1883;
    const char* MQTT_BROKER = "192.168.1.22"; 

    void publish_(const char* topic, float value);
  
  public:
    BL0940PubClient(Client& client) : PubSubClient(client) {};

    const uint16_t getMqttPort() { return MQTT_PORT; };
    const char* getMqttBroker() { return MQTT_BROKER; };

    void connectToMQTTBroker();
    
    void publishVoltageRMS(float voltageRMS);
    void publishCurrentRMS(float currentRMS);
    void publishActivePower(float activePower);
    void publishActiveEnergy(float activeEnergy);
    void publishPowerFactor(float powerFactor);
    void publishTemperature(float temperaute);
};

#endif /* BL0940_MQTT */