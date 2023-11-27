#include "radio.h"

void setup()
{
	Serial.begin(9600);  // Start up serial
	rfBegin(20);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)
	
	Serial.println("Printing bytes received via radio.");
}

void loop()
{
	uint8_t b[256];
	int len;
	
	if (len = rfAvailable())  // If serial comes in...
	{
		rfRead(b, 10);
		b[len] = 0;
		Serial.print("Received :");
		Serial.write((char*)b);
    Serial.println("");
	}

  if (len = Serial.available()) {
    for (int i = 0; i < len; i++) {
      b[i] = Serial.read(); 
    }
    rfWrite(b, len); // ...send it out the radio.
  }
	
}
