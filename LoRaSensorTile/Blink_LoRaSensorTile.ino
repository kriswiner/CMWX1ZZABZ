/* LED Blink, for LoRa Sensor Tile
 
   This example code is in the public domain.
*/
#include <Arduino.h>
#include <STM32L0.h>

#define myLed 10 // blue led

float VDDA, Temperature;
uint32_t UID[3] = {0, 0, 0};

void setup() 
{
  Serial.begin(38400);
  delay(2000);
  Serial.println("Serial enabled!");
 
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with leds off, since active LOW

  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX); 
}

void loop() 
{
  digitalWrite(myLed, LOW);  // toggle blue led on
  delay(100);                // wait 100 milliseconds
  digitalWrite(myLed, HIGH); // toggle blue led off
  delay(1000);               // wait 1000 milliseconds
  
  VDDA = STM32L0.getVREF();
  Temperature = STM32L0.getTemperature();

  Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(Temperature, 2);
  
//  STM32L0.stop(2000);
}

