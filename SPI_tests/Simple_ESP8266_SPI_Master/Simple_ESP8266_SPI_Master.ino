
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
#include <SPI.h>

uint8_t _ss_pin = SS;

uint8_t data;

void _pulseSS()
{
  digitalWrite(_ss_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_ss_pin, LOW);
}

void setup()
{
  Serial.begin(115200);
  SPI.begin();

  pinMode(_ss_pin, OUTPUT);
  _pulseSS();
}

void loop()
{
  _pulseSS();
  data = SPI.transfer(0x01);
  _pulseSS();
  Serial.print("Send: 0x01 - Receive: ");
  Serial.println(data);
  
  delay(1000);
}
