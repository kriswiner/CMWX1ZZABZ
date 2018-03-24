/* SPIFlash_Cicada.ino
Sketch by Kris Winer December 16, 2016

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

// 64 MBit (8 MByte) SPI Flash 32,767, 256-byte pages
#define csPin 25 // SPI Flash chip select pin (PH0)
#define myLed 10 // blue led active LOW

uint16_t page_number = 0xEFF;     // set the page mumber for flash page write
uint8_t  sector_number = 0;       // set the sector number for sector write
uint8_t  flashPage[256];          // array to hold the data for flash page write
uint32_t t_start = 0;

SPIFlash SPIFlash(csPin);


void setup(void)
{
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led off
  
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  // power up the chip
    SPIFlash.init();
    SPIFlash.powerUp();
    SPIFlash.getChipID(); 
    
/* Initialize the array to 0,1,2,3 etc.*/
  for(uint16_t i = 0; i < 256; i++) {
    flashPage[i] = i;
  }
  
/* Write the page to page_number - this page MUST be in the erased state*/
  Serial.print("Write page:  0x"); Serial.println(page_number, HEX);  
  t_start = micros();
  SPIFlash.flash_page_program(flashPage, page_number);
  t_start = micros() - t_start;
  Serial.print("time (us) = "); Serial.println(t_start);

/* Read back page_number and print its contents which should be 0,1,2,3...*/
  Serial.print("Read Page 0x"); Serial.println(page_number, HEX);
  t_start = micros();
  SPIFlash.flash_read_pages(flashPage, page_number, 1);
  t_start = micros() - t_start;
  Serial.print("time (us) = "); Serial.println(t_start);
  
  for(uint16_t i = 0; i < 256; i++) {
    Serial.print(" 0x"); Serial.print(flashPage[i], HEX);
	if (i % 16==0) Serial.println();
  }
  Serial.println("");
  
/* Erase the entire chip which includes page_number*/
  t_start = millis();
  SPIFlash.flash_chip_erase(true);
  t_start = millis() - t_start;
  Serial.print("time (ms) = "); Serial.println(t_start);

/* Now read back the page. It should now be all 0xFF*/
  Serial.print( "Read Page 0x"); Serial.println(page_number, HEX);
  t_start = micros();
  SPIFlash.flash_read_pages(flashPage, page_number, 1);
  t_start = micros() - t_start;

  Serial.print("time (us) = "); Serial.println(t_start);
  for(uint16_t i = 0; i < 256; i++) {
    Serial.print(" 0x"); Serial.print(flashPage[i], HEX);
	if (i % 16==0) Serial.println();
  }
  Serial.println("");
  Serial.flush();

  digitalWrite(myLed, LOW); // when done turn led off
}

void loop(void)
{
}


