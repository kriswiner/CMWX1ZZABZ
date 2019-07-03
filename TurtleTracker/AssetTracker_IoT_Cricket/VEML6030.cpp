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
#include "VEML6030.h"
#include "I2Cdev.h"

VEML6030::VEML6030(I2Cdev* i2c_bus)
{
 _i2c_bus = i2c_bus;
}


void VEML6030::init(uint8_t IT, uint8_t Gain, uint8_t Persistance)
{
  // byte structure is (Low Byte, High Byte)
  // High byte, set gain and integration time high bits, 
  // Low byte, set integration time low bits, set persistance, enable interrupt, set ALS power on
  uint8_t data[2] = {((IT & 0x03) << 6) | Persistance << 4 | 0x02, (Gain << 3) | ((IT & 0x0C) >> 2)};  
  _i2c_bus->writeBytes(VEML6030_ADDRESS, VEML6030_ALS_CONF, 2, &data[0]);
}


uint16_t  VEML6030::getALSData()
{
    uint8_t rawData[2] = {0, 0};
    _i2c_bus->readBytes(VEML6030_ADDRESS, VEML6030_ALS, 2, &rawData[0]);
    return ((uint16_t) rawData[1] << 8) | rawData[0];
}


uint16_t  VEML6030::getWhiteData()
{
    uint8_t rawData[2] = {0, 0};
    _i2c_bus->readBytes(VEML6030_ADDRESS, VEML6030_WHITE, 2, &rawData[0]);
    return ((uint16_t) rawData[1] << 8) | rawData[0];
}


uint16_t VEML6030::getIntStatus()
{
    uint8_t rawData[2] = {0, 0};
    _i2c_bus->readBytes(VEML6030_ADDRESS, VEML6030_ALS_INT, 2, &rawData[0]);
    return (((uint16_t) rawData[1]) << 8) | rawData[0];
}


uint16_t VEML6030::getHighThreshold()
{
    uint8_t rawData[2] = {0, 0};
    _i2c_bus->readBytes(VEML6030_ADDRESS, VEML6030_ALS_WH, 2, &rawData[0]);
    return (((uint16_t) rawData[1]) << 8) | rawData[0];
}


uint16_t  VEML6030::getLowThreshold()
{
    uint8_t rawData[2] = {0, 0};
    _i2c_bus->readBytes(VEML6030_ADDRESS, VEML6030_ALS_WL, 2, &rawData[0]);
    return ((uint16_t) rawData[1] << 8) | rawData[0];
}


void VEML6030::setLowThreshold(uint16_t Threshold)
{
    uint8_t data[2] = {(Threshold & 0x00FF), (Threshold & 0xFF00) >> 8};
    _i2c_bus->writeBytes(VEML6030_ADDRESS, VEML6030_ALS_WL, 2, &data[0]);
}


void VEML6030::setHighThreshold(uint16_t Threshold)
{
    uint8_t data[2] = {Threshold & 0x00FF, (Threshold & 0xFF00) >> 8};
    _i2c_bus->writeBytes(VEML6030_ADDRESS, VEML6030_ALS_WH, 2, &data[0]);
}


void VEML6030::enablepowerSave(uint8_t powerMode)
{

    uint8_t data[2] = {powerMode << 1 | 0x01, 0x00};
    _i2c_bus->writeBytes(VEML6030_ADDRESS, VEML6030_PWR_SAVE, 2, &data[0]);
}

void VEML6030::disablepowerSave()
{
    uint8_t data[2] = {0x00, 0x00};
    _i2c_bus->writeBytes(VEML6030_ADDRESS, VEML6030_PWR_SAVE, 2, &data[0]);
}


void VEML6030::enable()
{
  uint8_t rawData[2] = {0, 0};
  _i2c_bus->readBytes(VEML6030_ADDRESS, VEML6030_ALS_CONF, 2, &rawData[0]); // read existing configuration data
  uint8_t data[2] = {rawData[0] & ~(0x01), rawData[1]}; // ALS power on
  _i2c_bus->writeBytes(VEML6030_ADDRESS, VEML6030_ALS_CONF, 2, &data[0]);
}


void VEML6030::disable()
{
  uint8_t rawData[2] = {0, 0};
  _i2c_bus->readBytes(VEML6030_ADDRESS, VEML6030_ALS_CONF, 2, &rawData[0]); // read existing configuration data
  uint8_t data[2] = {rawData[0] | 0x01, rawData[1]}; // ALS power off
  _i2c_bus->writeBytes(VEML6030_ADDRESS, VEML6030_ALS_CONF, 2, &data[0]);
}
