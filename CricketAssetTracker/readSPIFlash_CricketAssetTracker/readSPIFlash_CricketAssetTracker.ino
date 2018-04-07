/* SPIFlash_Ladybug.ino
Sketch by Kris Winer December 16. 2016

License: Use this sketch any way you choose; if you like it, buy me a beer sometime

Purpose: Checks function of a variety of SPI NOR flash memory chips hosted by the STM32L4
Dragonfly (STM32L476), Butterfly (STM32L433), and Ladybug (STML432) development boards or their variants.

Sketch takes advantage of the SPI.beginTransaction/SPI.EndTransaction protocol for efficiency
and maximum speed.

Sketch based on the work of Pete (El Supremo) as follows:
 * Copyright (c) 2014, Pete (El Supremo), el_supremo@shaw.ca
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 */

#include <SPI.h>
#include "SPIFlash.h"

// Highest page number is 0xFFFF = 65535 for 128 Mbit flash
// Highest page number is 0x7FFF = 32768 for  64 Mbit flash
// Highest page number is 0x0EFF =  4095 for   8 Mbit flash
uint16_t max_page_number = 0x7FFF;
unsigned char flashPage[256];
uint16_t page_number = 0;
uint8_t sector_number = 0;
uint8_t buffer[4] = {0, 0, 0, 0};

uint32_t compHumidity, compTemp, compPress;   // pressure, humidity, and temperature raw count output for BME280
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280
uint8_t second, minute, hour, day, month, year;
uint16_t rawVbat;
float VDDA, VBAT;
uint16_t gpsAlt;
float   temperature;         // Stores the real internal chip temperature in degrees Celsius
float latitude, longitude;   // Stores CAM M8Q GPS position output

#define csPin 25

SPIFlash SPIFlash(csPin);

void setup(void)
{ 
  Serial.begin(115200);
  delay(16000);
  Serial.println("Serial enabled!");

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);

  // check SPI Flash ID
  SPIFlash.init();
  SPIFlash.powerUp();
  SPIFlash.getChipID();
 
  // read Sensor Tile SPI flash
  for(page_number = 0; page_number < 1000; page_number++)  {

   //  Serial.print("Read Page 0x"); Serial.println(page_number, HEX);
   SPIFlash.flash_read_pages(flashPage, page_number, 1);
      
   for(sector_number = 0; sector_number < 8; sector_number++) {
    
    // reconstruct latitude
    buffer[0] = flashPage[sector_number*32 + 0];
    buffer[1] = flashPage[sector_number*32 + 1];
    buffer[2] = flashPage[sector_number*32 + 2];
    buffer[3] = flashPage[sector_number*32 + 3];

    latitude = (float) ( (int32_t) (buffer[0] << 24) | (int32_t) (buffer[1] << 16) | (int32_t) (buffer[2] << 8) | (int32_t) buffer[4]);
    latitude /= 10000000.0f;
    
    // reconstruct longitude
    buffer[0] = flashPage[sector_number*32 + 4];
    buffer[1] = flashPage[sector_number*32 + 5];
    buffer[2] = flashPage[sector_number*32 + 6];
    buffer[3] = flashPage[sector_number*32 + 7];

    longitude = (float) ( (int32_t) (buffer[0] << 24) | (int32_t) (buffer[1] << 16) | (int32_t) (buffer[2] << 8) | (int32_t) buffer[4]);
    longitude /= 10000000.0f;

    // reconstruct temperature
    buffer[0] = flashPage[sector_number*32 + 8];
    buffer[1] = flashPage[sector_number*32 + 9];
    buffer[2] = flashPage[sector_number*32 + 10];
    buffer[3] = flashPage[sector_number*32 + 11];

    temperature_C = (float) ( (int32_t) (buffer[0] << 24) | (int32_t) (buffer[1] << 16) | (int32_t) (buffer[2] << 8) | (int32_t) buffer[4]);
    temperature_C /= 100.0f; // temperature in degree Centigrade
    
    // reconstruct humidity
    buffer[0] = flashPage[sector_number*32 + 12];
    buffer[1] = flashPage[sector_number*32 + 13];
    buffer[2] = flashPage[sector_number*32 + 14];
    buffer[3] = flashPage[sector_number*32 + 15];

    humidity = (float) ( (int32_t) (buffer[0] << 24) | (int32_t) (buffer[1] << 16) | (int32_t) (buffer[2] << 8) | (int32_t) buffer[4]);
    humidity /= 1024.0f; // humidity on %rH
    
    // reconstruct pressure
    buffer[0] = flashPage[sector_number*32 + 16];
    buffer[1] = flashPage[sector_number*32 + 17];
    buffer[2] = flashPage[sector_number*32 + 18];
    buffer[3] = flashPage[sector_number*32 + 19];

    pressure = (float) ( (int32_t) (buffer[0] << 24) | (int32_t) (buffer[1] << 16) | (int32_t) (buffer[2] << 8) | (int32_t) buffer[4]);
    pressure /= 25600.0f;  // pressure in millibars
    
    second =  flashPage[sector_number*32 + 20];
    minute =  flashPage[sector_number*32 + 21];
    hour =    flashPage[sector_number*32 + 22];
    day =     flashPage[sector_number*32 + 23];
    month =   flashPage[sector_number*32 + 24];
    year =    flashPage[sector_number*32 + 25];

    gpsAlt = ((uint16_t) flashPage[sector_number*32 + 26] << 8) |  flashPage[sector_number*32 + 27];
    
    rawVbat = ((uint16_t) flashPage[sector_number*32 + 28] << 8) |  flashPage[sector_number*32 + 29];

    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
    
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   

    gpsAlt *= 3.2808399f/10.0f;

    VBAT = rawVbat/100.0f;

    // Output for spreadsheet analysis
    if(month < 10) {Serial.print("0"); Serial.print(month);} else Serial.print(month);
    Serial.print("/");Serial.print(day); Serial.print("/");Serial.print(year); Serial.print(" ");
    if(hour < 10) {Serial.print("0"); Serial.print(hour);} else Serial.print(hour);
    Serial.print(":"); 
    if(minute < 10) {Serial.print("0"); Serial.print(minute);} else Serial.print(minute); 
    Serial.print(":"); 
    if(second < 10) {Serial.print("0"); Serial.print(second);} else Serial.print(second); Serial.print(",");      
 
    Serial.print(pressure, 2); Serial.print(","); Serial.print(temperature_C, 2); Serial.print(",");Serial.print(humidity, 2); Serial.print(",");
    Serial.print(latitude, 7); Serial.print(","); Serial.print(longitude, 7); Serial.print(",");    
    Serial.print(VBAT, 2);  Serial.print(","); Serial.print(gpsAlt); Serial.print(","); Serial.println(altitude, 2);  
    }
  
  }


}

void loop(void)
{
}



