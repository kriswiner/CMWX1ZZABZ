/* 07/01/2019 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 The MS5837-30BA is a new generation of  high resolution pressure sensors with I2C bus interface for     
 depth measurement systems with a water depth resolution of 2  mm. The sensor module includes a high linearity  
 pressure sensor and an ultra-lowpower 24 bit ΔΣADC with internal factory calibrated coefficients.  It provides  
 a precise digital 24 Bit pressure and temperature value and different operation modes that allow the user to 
 optimize for conversion speed and current consumption.  A high resolution temperature output allows the 
 implementation in depth measurement systems and thermometer function without any additional sensor.   
 The MS5837-30BA can be interfaced to virtually any microcontroller. The communication protocol is simple, 
 without the need of programming internal registers in the device.  The gel protection and antimagnetic stainless
 steel cap make the module water resistant.    
 
 Library may be used freely and without limit with attribution.
 
*/
  
#ifndef MS5837_h
#define MS5837_h

#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"

//
////////////////////////////
// MS5837 Command Codes //
////////////////////////////
// See MS5837-302BA Low Voltage Barometric Pressure Sensor Data Sheet
// http://www.mouser.com/ds/2/418/MS5837-30BA-736494.pdf
#define MS5837_RESET      0x1E
#define MS5837_CONVERT_D1 0x40
#define MS5837_CONVERT_D2 0x50
#define MS5837_ADC_READ   0x00

#define MS5837_ADDRESS 0x76   // Address of altimeter

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50

class MS5837
{
  public: 
  MS5837(I2Cdev* i2c_bus);
  void Reset();
  void PromRead(uint16_t * destination);
  uint32_t DataRead(uint8_t CMD, uint8_t OSR);
  unsigned char checkCRC(uint16_t * n_prom);
  private:
  I2Cdev* _i2c_bus;
  };

#endif
