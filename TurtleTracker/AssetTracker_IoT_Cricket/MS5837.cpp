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
  
#include "MS5837.h"
#include "I2Cdev.h"

MS5837::MS5837(I2Cdev* i2c_bus)
{
 _i2c_bus = i2c_bus;
}


void MS5837::Reset()
{
 _i2c_bus->writeCommand(MS5837_ADDRESS, MS5837_RESET, true);
 }


uint32_t MS5837::DataRead(uint8_t CMD, uint8_t OSR)
{
        uint8_t data[3] = {0,0,0};
        _i2c_bus->writeCommand(MS5837_ADDRESS, CMD | OSR, false);
        
        switch (OSR)
        {
          case ADC_256: delay(1); break;         // delay for conversion to complete
          case ADC_512: delay(3); break;
          case ADC_1024: delay(4); break;
          case ADC_2048: delay(6); break;
          case ADC_4096: delay(10); break;
          case ADC_8192: delay(20); break;
        }
     
         _i2c_bus->readBytes(MS5837_ADDRESS, MS5837_ADC_READ, 3, &data[0]);// Put read results in the Rx buffer
         return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
        }


void MS5837::PromRead(uint16_t * destination)
{
        uint8_t data[2] = {0,0};
        for (uint8_t ii = 0; ii < 7; ii++) 
        {
          _i2c_bus->readBytes(MS5837_ADDRESS, 0xA0 | ii << 1, 2, &data[0]);// Put read results in the Rx buffer
          destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
        }
}


unsigned char MS5837::checkCRC(uint16_t * n_prom)
{
  int cnt;
  unsigned int n_rem = 0;
  unsigned char n_bit;
  
  n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
  n_prom[7] = 0;
  for(cnt = 0; cnt < 16; cnt++)
  {
    if(cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
    else         n_rem ^= (unsigned short)  (n_prom[cnt>>1]>>8);
    for(n_bit = 8; n_bit > 0; n_bit--)
    {
        if(n_rem & 0x8000)    n_rem = (n_rem<<1) ^ 0x3000;
        else                  n_rem = (n_rem<<1);
    }
  }
  n_rem = ((n_rem>>12) & 0x000F);
  return (n_rem ^ 0x00);
}
