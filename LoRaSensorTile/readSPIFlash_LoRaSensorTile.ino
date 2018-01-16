/* readSPIFlash_LoRaSensorTile.ino
 *  
Sketch by Kris Winer January 14, 2018

License: Use this sketch any way you choose; if you like it, buy me a beer sometime

Purpose: Reads 16 MByte SPI NOR flash on LoRa SensorTile and reconstructs bytes into 
spreadsheet-readable scaled sensor data

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
// Highest page number is 0x7FFF = 32767 for  64 Mbit flash
// Highest page number is 0x0EFF =  4095 for   8 Mbit flash
uint16_t max_page_number = 0xFFFF;
unsigned char flashPage[256];
uint16_t page_number = 0;
uint8_t sector_number = 0;

uint32_t compHumidity, compTemp, compPress;   // pressure, humidity, and temperature raw count output for BME280
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
uint16_t rawBat;
float VDDA, VBAT;
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float aRes = 2.0f/8192.0f;   // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
/*Choices are:
 IT_40  40 ms, IT_80 80 ms, IT_160  160 ms, IT_320  320 ms, IT_640 640 ms, IT_1280 1280 ms*/
uint8_t IT = 0;  // integration time variable IT_40
uint8_t ITime = 40;  // integration time in milliseconds
uint16_t RGBWData[4] = {0, 0, 0, 0};
float GSensitivity = 0.25168/((float) (1 << IT)); // ambient light sensitivity increases with integration time


#define csPin 21

SPIFlash SPIFlash(csPin);

void setup(void)
{ 
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  // check SPI Flash ID
  SPIFlash.SPIFlashinit();
  SPIFlash.getChipID();
 
  // read Sensor Tile SPI flash
  for(page_number = 0; page_number < 10; page_number++)  {

   //  Serial.print("Read Page 0x"); Serial.println(page_number, HEX);
   SPIFlash.flash_read_pages(flashPage, page_number, 1);
      
   for(sector_number = 0; sector_number < 7; sector_number++) {

    accelCount[0] = ((int16_t) flashPage[sector_number*36 + 1] << 8) | flashPage[sector_number*36 + 2];
    accelCount[1] = ((int16_t) flashPage[sector_number*36 + 3] << 8) | flashPage[sector_number*36 + 4];
    accelCount[2] = ((int16_t) flashPage[sector_number*36 + 5] << 8) | flashPage[sector_number*36 + 6];
    
    ax = (float)accelCount[0]*aRes/4.0f;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes/4.0f;   
    az = (float)accelCount[2]*aRes/4.0f;  

    int16_t tmp = ((int16_t) (flashPage[sector_number*36 + 0] << 8   | 0x00)) >> 8;  // reconstruct signed 8-bit accel temperature
    temperature = 0.5f * ((float) tmp )  + 23.0f; // Accel chip temperature in degrees Centigrade
    
    compTemp = ((uint32_t) flashPage[sector_number*36 + 7] << 24) |  ((uint32_t)flashPage[sector_number*36 + 8] << 16) |  ((uint32_t)flashPage[sector_number*36 + 9] << 8) | flashPage[sector_number*36 + 10];
    compHumidity = ((uint32_t) flashPage[sector_number*36 + 11] << 24) |  ((uint32_t)flashPage[sector_number*36 + 12] << 16) |  ((uint32_t)flashPage[sector_number*36 + 13] << 8) | flashPage[sector_number*36 + 14];
    compPress = ((uint32_t) flashPage[sector_number*36 + 15] << 24) |  ((uint32_t)flashPage[sector_number*36 + 16] << 16) |  ((uint32_t)flashPage[sector_number*36 + 17] << 8) | flashPage[sector_number*36 + 18];

    Seconds = flashPage[sector_number*36 + 19];
    Minutes = flashPage[sector_number*36 + 20];
    Hours = flashPage[sector_number*36 + 21];
    Day = flashPage[sector_number*36 + 22];
    Month = flashPage[sector_number*36 + 23];
    Year = flashPage[sector_number*36 + 24];

    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    pressure = (float) compPress/25600.0f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH

    RGBWData[0] = ((uint16_t) flashPage[sector_number*36 + 27] << 8) |  flashPage[sector_number*36 + 28];
    RGBWData[1] = ((uint16_t) flashPage[sector_number*36 + 29] << 8) |  flashPage[sector_number*36 + 30];
    RGBWData[2] = ((uint16_t) flashPage[sector_number*36 + 31] << 8) |  flashPage[sector_number*36 + 32];
    RGBWData[3] = ((uint16_t) flashPage[sector_number*36 + 33] << 8) |  flashPage[sector_number*36 + 34];


    // Empirical estimation of the correlated color temperature CCT:
    // see https://www.vishay.com/docs/84331/designingveml6040.pdf
      float temp = ( (float) (RGBWData[0] - RGBWData[2])/(float) RGBWData[1] );
      float CCT = 4278.6f*powf(temp, -1.2455f) + 0.5f;

    rawBat = ((uint16_t) flashPage[sector_number*36 + 25] << 8) |  flashPage[sector_number*36 + 26];
    VBAT = (1270.0f/1000.0f) * 3.30f * ((float)rawBat)/4095.0f;

    // Output for spreadsheet analysis
    if(Month < 10) {Serial.print("0"); Serial.print(Month);} else Serial.print(Month);
    Serial.print("/");Serial.print(Day); Serial.print("/");Serial.print(Year); Serial.print(" ");
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.print(Seconds);} else Serial.print(Seconds); Serial.print(",");      
    Serial.print(pressure, 2); Serial.print(","); Serial.print(temperature_C, 2); Serial.print(",");Serial.print(temperature_F, 2); Serial.print(",");
    Serial.print(altitude, 2); Serial.print(","); Serial.print(humidity, 1); Serial.print(","); 
    Serial.print(VBAT, 2); Serial.print(",");
    Serial.print((float)RGBWData[0]/96.0f, 2); Serial.print(",");
    Serial.print((float)RGBWData[1]/74.0f, 2); Serial.print(",");
    Serial.print((float)RGBWData[2]/56.0f, 2); Serial.print(",");
    Serial.print((float)RGBWData[1]*GSensitivity, 2); Serial.print(",");
    Serial.print(RGBWData[3] - RGBWData[0] - RGBWData[1] - RGBWData[2]); Serial.print(",");
    Serial.print(CCT,2); Serial.print(",");
    Serial.print((int)1000*ax); Serial.print(","); Serial.print((int)1000*ay); Serial.print(","); Serial.print((int)1000*az); Serial.print(","); Serial.println(temperature, 2); 

    }
  
  }


}

void loop(void)
{
}
