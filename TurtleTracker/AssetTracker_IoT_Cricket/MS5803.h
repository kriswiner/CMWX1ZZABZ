/* 07/01/2019 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 The MS5803-14BA is a new generation of high resolution pressure sensors with SPI and I2C bus interface. 
 It is optimized for depth measurement systems with a water depth resolution of 1cm and below. The sensor 
 module includes  a  high  linear  pressure  sensor  and  an  ultra  low  power  24  bit  ΔΣ  ADC  with  
 internal  factory  calibrated coefficients.  It  provides  a  precise  digital  24  Bit  pressure  and  
 temperature  value  and  different  operation  modes  that  allow  the  user  to  optimize  for  conversion  
 speed  and  current  consumption.  A  high  resolution  temperature output  allows  the  implementation  of  
 a  depth  measurement  systems  and  thermometer  function  without  any  additional  sensor.  
 The  MS5803-14BA  can  be  interfaced  to  any  microcontroller.  The  communication  protocol  is  simple,  
 without  the  need  to  programming  internal  registers  in  the  device.  The  gel  protection  and  
 antimagnetic  stainless steel cap protects against 30 bar overpressure water resistant.     
 
 Library may be used freely and without limit with attribution.
 
*/
  
#ifndef MS5803_h
#define MS5803_h

#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"

//
////////////////////////////
// MS5803 Command Codes //
////////////////////////////
// See MS5803-302BA Low Voltage Barometric Pressure Sensor Data Sheet
// http://www.mouser.com/ds/2/418/MS5803-30BA-736494.pdf
#define MS5803_RESET      0x1E
#define MS5803_CONVERT_D1 0x40
#define MS5803_CONVERT_D2 0x50
#define MS5803_ADC_READ   0x00

#define MS5803_ADDRESS 0x77   // Address of altimeter

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08

#define ADC_D1   0x40
#define ADC_D2   0x50

class MS5803
{
  public: 
  MS5803(I2Cdev* i2c_bus);
  void Reset();
  void PromRead(uint16_t * destination);
  uint32_t DataRead(uint8_t CMD, uint8_t OSR);
  unsigned char checkCRC(uint16_t * n_prom);
  private:
  I2Cdev* _i2c_bus;
  };

#endif
