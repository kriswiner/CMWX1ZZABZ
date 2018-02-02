/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 VEML6040 color sensor senses red, green, blue, and white light and incorporates photodiodes, amplifiers, 
 and analog / digital circuits into a single chip using CMOS process. With the   color   sensor   applied,   
 the   brightness,   and   color temperature of backlight can be adjusted base on ambient light  source  
 that  makes  panel  looks  more  comfortable  for  end   user’s   eyes.   VEML6040’s   adoption   of   FiltronTM
 technology  achieves  the  closest  ambient  light  spectral  sensitivity to real human eye responses.

 VEML6040  provides  excellent  temperature  compensation  capability  for  keeping  the  output  stable  
 under  changing  temperature.   VEML6040’s   function   are   easily   operated   via the simple command format 
 of I2C (SMBus compatible) interface  protocol.  VEML6040’s  operating  voltage  ranges  from   2.5   V   to   
 3.6   V.   VEML6040   is   packaged   in   a   lead  (Pb)-free  4  pin  OPLGA  package  which  offers  the  best market-proven reliability.
 
 Library may be used freely and without limit with attribution.
 
*/
  
#ifndef VEML6040_h
#define VEML6040_h

#include "Arduino.h"
#include "Wire.h"

// http://www.mouser.com/pdfdocs/veml6040.PDF
////////////////////////////
// VEML6040 Command Codes //
////////////////////////////
#define  VEML6040_CONF            0x00 // command codes
#define  VEML6040_R_DATA          0x08  
#define  VEML6040_G_DATA          0x09 
#define  VEML6040_B_DATA          0x0A
#define  VEML6040_W_DATA          0x0B

#define VEML6040_ADDRESS          0x10

#define  IT_40   0  //   40 ms
#define  IT_80   1  //   80 ms
#define  IT_160  2  //  160 ms
#define  IT_320  3  //  320 ms
#define  IT_640  4  //  640 ms
#define  IT_1280 5  // 1280 ms

class VEML6040
{
  public: 
  VEML6040();
  uint16_t  getRGBWdata(uint16_t * destination);
  void enableVEML6040(uint8_t IT);
  void disableVEML6040(uint8_t IT);
  void I2Cscan();
  private:
  };

#endif
