
/*
    SPI Safe Master Demo Sketch
    Connect the SPI Master device to the following pins on the esp8266:

    GPIO    NodeMCU   Name  |   Uno
   ===================================
     15       D8       SS   |   D10
     13       D7      MOSI  |   D11
     12       D6      MISO  |   D12
     14       D5      SCK   |   D13

    Note: If the ESP is booting at a moment when the SPI Master has the Select line HIGH (deselected)
    the ESP8266 WILL FAIL to boot!
    This sketch tries to go around this issue by only pulsing the Slave Select line to reset the command
    and keeping the line LOW all other time.

*/

#include "BL0940_SPI.hpp"

BL0940_SPI bl0940(SS);

void setup()
{
  Serial.begin(115200);
  delay(100);

  Serial.println("");
  Serial.println("[----- BL0940 SPI TEST -----]");

  bl0940.Reset();
  bl0940.setFrequency(50); //60[Hz]
  bl0940.setUpdateRate(800); //800[ms]
}

void loop()
{
  float voltage;
  bl0940.getVoltageRMS( &voltage );
  Serial.printf("%.2f [V]\n", voltage );

  float current;
  bl0940.getCurrentRMS( &current );
  Serial.printf("%.3f [A]\n", current );

  float activePower;
  bl0940.getActivePower( &activePower );
  Serial.printf("%.2f [W]\n", activePower );

  float activeEnergy;
  bl0940.getActiveEnergy( &activeEnergy );
  Serial.printf("%.4f [kWh]\n", activeEnergy );

  float powerFactor;
  bl0940.getPowerFactor( &powerFactor );
  Serial.printf("%.1f [%%]\n", powerFactor );

  float temperature;
  bl0940.getTemperature( &temperature );
  Serial.printf("%.2f [deg C]\n", temperature );

  Serial.println("");
  delay(1000);
}
