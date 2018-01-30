/* Simple getDevEUI

   This example code is in the public domain.
*/

#include "LoRaWAN.h"

char buffer[32];

void setup( void ) {
  Serial1.begin(9600);
  LoRaWAN.begin(EU868);
  Serial1.println("devEUI request");
  LoRaWAN.getDevEui(buffer,18);
  Serial1.println(buffer);
}

void loop( void ) {
}
