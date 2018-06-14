/* LED Blink, for Grasshopper
 
   This example code is in the public domain.
*/
#include <STM32L0.h>

#define myLed 13 // red led

float VDDA, VBAT, VUSB, Temperature;
uint32_t UID[3] = {0, 0, 0};
char buffer[32];

void setup() 
{
  Serial.begin(38400);
  delay(2000);
  Serial.println("Serial enabled!");
 
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);  // start with leds off, since active HIGH
  
  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX); 
  LoRaWAN.getDevEui(buffer, 18);
  Serial.print("STM32L0 Device EUI = "); Serial.println(buffer); 
  }

void loop() 
{
  digitalWrite(myLed, HIGH); // toggle red led on
  delay(100);                // wait 100 millisecond
  digitalWrite(myLed, LOW);  // toggle red led off
  delay(1000);               // wait 1000 milliseconds
  
  VDDA = STM32L0.getVDDA();
  VUSB = STM32L0.getVBUS();
  Temperature = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  if(VUSB ==  1)  Serial.println("USB Connected!"); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(Temperature, 2);
  
  STM32L0.stop(5000);
}

