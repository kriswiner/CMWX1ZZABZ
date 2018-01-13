/* LED Blink, for Grasshopper
 
   This example code is in the public domain.
*/
#include <Arduino.h>
#include <STM32L0.h>

#define myLed 13 // red led

float VDDA, Temperature;
uint32_t UID[3] = {0, 0, 0};

void setup() 
{
  Serial.begin(38400);
  delay(2000);
  Serial.println("Serial enabled!");
 
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);  // start with leds off, since active HIGH

  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX); 
  }

void loop() 
{
  digitalWrite(myLed, HIGH); // toggle red led on
  delay(100);                // wait 100 millisecond
  digitalWrite(myLed, LOW);  // toggle red led off
  delay(1000);
  
  VDDA = STM32L0.getVREF();
  Temperature = STM32L0.getTemperature();

  Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(Temperature, 2);
  
 // STM32L0.stop(2000);
}
