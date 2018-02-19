/* DAC and ADC example, for Grasshopper
 
   This example code is in the public domain.
*/
#include <STM32L0.h>
#include "LoRaWAN.h"

#define myLed 13 // red led

float VDDA, VBUS, Temperature;
uint32_t UID[3] = {0, 0, 0};
char buffer[32];

#define myADC A2
#define myDAC A0

float DACvalue = 0.0f, ADCvalue = 0.0f;

void setup() 
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("Serial enabled!");
 
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);  // start with leds off, since active HIGH
  
  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX); 
  LoRaWAN.getDevEui(buffer, 18);
  Serial.print("STM32L0 Device EUI = "); Serial.println(buffer); 

  pinMode(myADC, INPUT);
  analogReadResolution(12); // 12-bit ADC instead of 8-bit default
  pinMode(myDAC, OUTPUT);

}

void loop() 
{
  digitalWrite(myLed, HIGH); // toggle red led on
  delay(100);                // wait 100 millisecond
  digitalWrite(myLed, LOW);  // toggle red led off
  delay(1000);               // wait 1000 milliseconds
  
  VDDA = STM32L0.getVDDA();
  VBUS = STM32L0.getVBUS();
  Temperature = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  if(VBUS ==  1)  Serial.println("USB Connected!"); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(Temperature, 2);

  for (int ii = 0; ii <256; ii++)
  {
  analogWrite(myDAC, ii); // write/read voltage in 3.3/256 V increments every 100 milliseconds
  DACvalue = 3.30f * ((float) ii)/255.0f;  // should be written
  delay(100);
  ADCvalue = VDDA * ((float) analogRead(myADC))/4095.0f; // is actually read
  Serial.print("DAC output is "); Serial.print(DACvalue, 3); 
  Serial.print(" V, ADC input is "); Serial.print(ADCvalue, 3);Serial.println(" V");
//  Serial.print(ii); Serial.print(",");Serial.print(DACvalue);Serial.print(",");Serial.println(ADCvalue);
  }
  
 // STM32L0.stop(5000);
}
