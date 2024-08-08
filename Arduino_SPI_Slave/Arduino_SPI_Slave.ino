#include<SPI.h>

volatile boolean received;
volatile byte receivedData;

ISR (SPI_STC_vect)        //Inerrrput routine function 
{
  Serial.println(SPDR, HEX);                        
  SPDR = 0xFF;          
  // receivedData = SPDR;   // Get the received data from SPDR register
  // received = true;       // Sets received as True 
}

void setup()
{
  Serial.begin(115200);

  pinMode(MISO, OUTPUT);    //Sets MISO as OUTPUT
  SPCR |= _BV(SPE);         //Turn on SPI in Slave Mode
  received = false;
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  SPI.attachInterrupt();    //Activate SPI Interuupt 
}

void loop()
{ 
  if(received) {
    received = false;
  }
}