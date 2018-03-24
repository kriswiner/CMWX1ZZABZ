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
#include "BME280.h"
#include "Wire.h"

// Highest page number is 0xFFFF = 65535 for 128 Mbit flash
// Highest page number is 0x7FFF = 32767 for  64 Mbit flash
// Highest page number is 0x0EFF =  4095 for   8 Mbit flash
uint16_t max_page_number = 0x7FFF;
unsigned char flashPage[256];
uint16_t page_number = 0;
uint8_t sector_number = 0;

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

// BME280 definitions
/* Specify BME280 configuration
 *  Choices are:
 P_OSR_01, P_OSR_02, P_OSR_04, P_OSR_08, P_OSR_16    // pressure oversampling
 H_OSR_01, H_OSR_02, H_OSR_04, H_OSR_08, H_OSR_16    // humidity oversampling
 T_OSR_01, T_OSR_02, T_OSR_04, T_OSR_08, T_OSR_16    // temperature oversampling
 full, BW0_223ODR,BW0_092ODR, BW0_042ODR, BW0_021ODR // bandwidth at 0.021 x sample rate
 BME280Sleep, forced, forced2, normal //operation modes
 t_00_5ms = 0, t_62_5ms, t_125ms, t_250ms, t_500ms, t_1000ms, t_10ms, t_20ms // determines sample rate
 */
uint8_t Posr = P_OSR_01, Hosr = H_OSR_01, Tosr = T_OSR_01, Mode = Sleep, IIRFilter = full, SBy = t_1000ms;     // set pressure amd temperature output data rate

int32_t rawPress, rawTemp, rawHumidity;    // pressure, humidity, and temperature raw count output for BME280
double temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

BME280 BME280; // instantiate BME280 class

#define csPin 25

unsigned char r_page[256];

SPIFlash SPIFlash(csPin);

void setup(void)
{ 
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);

  Wire.begin(); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);

  BME280.I2Cscan(); // should detect BME280 at 0x77, BMA280 at 0x18, and VEML6040 at 0x10
  delay(1000);

  BME280.resetBME280();                                              // reset BME280 before initilization
  delay(100);

  BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy);         // Initialize BME280 altimeter
  BME280.BME280forced();                                             // get initial data sample, then go back to sleep


  // check SPI Flash ID
  SPIFlash.SPIFlashinit();
  SPIFlash.powerUp();
  SPIFlash.getChipID();
 
  // read Sensor Tile SPI flash
  for(page_number = 0; page_number < 20; page_number++)  {

   //  Serial.print("Read Page 0x"); Serial.println(page_number, HEX);
   SPIFlash.flash_read_pages(flashPage, page_number, 1);
      
   for(sector_number = 0; sector_number < 9; sector_number++) {
   
    rawTemp = (int32_t) ((uint32_t) flashPage[sector_number*28 + 0] << 24) |  ((uint32_t)flashPage[sector_number*28 + 1] << 16) |  ((uint32_t)flashPage[sector_number*28 + 2] << 8) | flashPage[sector_number*28 + 3];
    rawHumidity = (int32_t) ((uint32_t) flashPage[sector_number*28 + 4] << 24) |  ((uint32_t)flashPage[sector_number*28 + 5] << 16) |  ((uint32_t)flashPage[sector_number*28 + 6] << 8) | flashPage[sector_number*28 + 7];
    rawPress = (int32_t) ((uint32_t) flashPage[sector_number*28 + 8] << 24) |  ((uint32_t)flashPage[sector_number*28 + 9] << 16) |  ((uint32_t)flashPage[sector_number*28 + 10] << 8) | flashPage[sector_number*28 + 11];

    temperature_C = BME280.BME280_compensate_T_double(rawTemp);
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    pressure  = BME280.BME280_compensate_P_double(rawPress)/100.0f;
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    humidity    = BME280.BME280_compensate_H_double(rawHumidity); // Humidity in %RH
   
    Seconds = flashPage[sector_number*28 + 12];
    Minutes = flashPage[sector_number*28 + 13];
    Hours =   flashPage[sector_number*28 + 14];
    Day =     flashPage[sector_number*28 + 15];
    Month =   flashPage[sector_number*28 + 16];
    Year =    flashPage[sector_number*28 + 17];


    RGBWData[0] = ((uint16_t) flashPage[sector_number*28 + 20] << 8) |  flashPage[sector_number*28 + 21];
    RGBWData[1] = ((uint16_t) flashPage[sector_number*28 + 22] << 8) |  flashPage[sector_number*28 + 23];
    RGBWData[2] = ((uint16_t) flashPage[sector_number*28 + 24] << 8) |  flashPage[sector_number*28 + 25];
    RGBWData[3] = ((uint16_t) flashPage[sector_number*28 + 26] << 8) |  flashPage[sector_number*28 + 27];


    // Empirical estimation of the correlated color temperature CCT:
    // see https://www.vishay.com/docs/84331/designingveml6040.pdf
      float temp = ( (float) (RGBWData[0] - RGBWData[2])/(float) RGBWData[1] );
      float CCT = 4278.6f*powf(temp, -1.2455f) + 0.5f;

    rawBat = ((uint16_t) flashPage[sector_number*28 + 18] << 8) |  flashPage[sector_number*28 + 19];
    VBAT = ((float)rawBat)/100.0f;

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
    Serial.println(CCT,2);  

    }
  
  }


}

void loop(void)
{
}


