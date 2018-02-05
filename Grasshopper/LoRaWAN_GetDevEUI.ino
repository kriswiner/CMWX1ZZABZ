/* Simple getDevEUI

   This example code is in the public domain.
*/

#include "LoRaWAN.h"

char buffer[32];

void setup( void ) {
  Serial.begin(9600);
  while (! Serial); // wait for serial port to connect. Needed for native USB
  LoRaWAN.begin(EU868);
  Serial.println("devEUI request");
  LoRaWAN.getDevEui(buffer, sizeof(buffer));
  Serial.println(buffer);
}

void loop( void ) {
}
