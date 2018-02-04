/* RTC demo sketch, for Grasshopper
 
   This example code is in the public domain.
*/
#include <Arduino.h>
#include <STM32L0.h>
#include <RTC.h>

#define myLed 13 // red led

float VDDA, Temperature;
uint32_t UID[3] = {0, 0, 0}; 

// RTC set up
/* Change these values to set the current initial time */

uint8_t seconds = 0;
uint8_t minutes = 51;
uint8_t hours = 10;

/* Change these values to set the current initial date */

uint8_t day = 14;
uint8_t month = 1;
uint8_t year = 18;

uint8_t Seconds, Minutes, Hours, Day, Month, Year, count = 0;

bool alarmFlag = false;

void setup() 
{
  Serial.begin(38400);
  delay(2000);
  Serial.println("Serial enabled!");
 
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // start with led off, since active HIGH

  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX);Serial.println(UID[2], HEX); 

  // Set the time
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);

  // set alarm to update the RTC every second
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch);

}

void loop() 
{

  if(alarmFlag)  { // update RTC output (serial display) whenever the RTC alarm condition is achieved
     alarmFlag = false;
  
  VDDA = STM32L0.getVDDA();
  Temperature = STM32L0.getTemperature();

  Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(Temperature, 2);

  // Read RTC
  Serial.println("RTC:");
  Day = RTC.getDay();
  Month = RTC.getMonth();
  Year = RTC.getYear();
  Seconds = RTC.getSeconds();
  Minutes = RTC.getMinutes();
  Hours   = RTC.getHours();     
  if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
  Serial.print(":"); 
  if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
  Serial.print(":"); 
  if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

  Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
  Serial.println(" ");
    
  digitalWrite(myLed, HIGH); delay(10); digitalWrite(myLed, LOW);

  STM32L0.stop();
  
  }
}

 void alarmMatch()
{
  alarmFlag = true; // Just set flag when interrupt received, don't try reading data in an interrupt handler
  STM32L0.wakeup();
}
