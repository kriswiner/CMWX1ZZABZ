/* 07/01/2019 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 VEML6030  is  a  high  accuracy  ambient  light  digital  16-bit  resolution sensor in a miniature transparent 
 2 mm x 2 mm package.  It  includes  a  high  sensitive  photodiode,  a  low  noise  amplifier,  a  16-bit  A/D  
 converter  and  supports  an  easy to use I2C bus communication interface and additional interrupt feature.

 VEML6030’s   functions   are   easily   operated   via the simple command format 
 of I2C (SMBus compatible) interface  protocol.  VEML6030’s  operating  voltage  ranges  from   2.5   V   to   
 3.6   V.    
 
 Library may be used freely and without limit with attribution.
 
*/
  
#ifndef VEML6030_h
#define VEML6030_h

#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"

// https://www.vishay.com/docs/84366/veml6030.pdf
// https://www.vishay.com/docs/84367/designingveml6030.pdf
////////////////////////////
// VEML6030 Command Codes //
////////////////////////////
#define  VEML6030_ALS_CONF        0x00 // command codes
#define  VEML6030_ALS_WH          0x01  
#define  VEML6030_ALS_WL          0x02 
#define  VEML6030_PWR_SAVE        0x03
#define  VEML6030_ALS             0x04
#define  VEML6030_WHITE           0x05
#define  VEML6030_ALS_INT         0x06

#define VEML6030_ADDRESS          0x10 // 0x10 when address pin LOW, 0x48 when HIGH

#define  IT_100   0x00  //   100 ms /ALS integration time setting
#define  IT_200   0x01  //   200 ms
#define  IT_400   0x02  //   400 ms
#define  IT_800   0x03  //   800 ms
#define  IT_50    0x08  //    50 ms
#define  IT_25    0x0C  //    25 ms

#define Gain_1x       0x00 // 1x gain
#define Gain_2x       0x01 // 2x gain
#define Gain_0_125x   0x02 // 1/8 x gain
#define Gain_0_25x    0x03 // 1/4 x gain

class VEML6030
{
  public: 
  VEML6030(I2Cdev* i2c_bus);
  void init(uint8_t IT, uint8_t Gain, uint8_t Persistance);
  uint16_t getALSData();
  uint16_t getWhiteData();
  void setHighThreshold(uint16_t Threshold);
  uint16_t getHighThreshold();
  void setLowThreshold(uint16_t Threshold);
  uint16_t getLowThreshold();
  uint16_t getIntStatus();
  void enablepowerSave(uint8_t powerMode);
  void disablepowerSave();
  void enable();
  void disable();
  private:
  I2Cdev* _i2c_bus;
  };

#endif
