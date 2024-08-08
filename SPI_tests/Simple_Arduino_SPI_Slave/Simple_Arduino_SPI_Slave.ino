#include<SPI.h>

volatile boolean received;
volatile byte receivedData;

ISR (SPI_STC_vect)        //Inerrrput routine function 
{
  receivedData = SPDR;   // Get the received data from SPDR register
  received = true;       // Sets received as True 
}

void setup()
{
  Serial.begin(115200);

  pinMode(MISO, OUTPUT);    //Sets MISO as OUTPUT
  SPCR |= _BV(SPE);         //Turn on SPI in Slave Mode
  received = false;
  SPI.attachInterrupt();    //Activate SPI Interuupt 
}

void loop()
{ 
  if(received) {
    Serial.print("Data received! Data: ");
    Serial.println(receivedData, HEX);                        
    SPDR = 0x02;    // Data to send
    received = false;
  }
}