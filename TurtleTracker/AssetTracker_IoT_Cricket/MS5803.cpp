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

 https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5803-01BA%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5803-01BA_B3.pdf%7FCAT-BLPS0038
 https://www.parallax.com/sites/default/files/downloads/29124-APPNote_520_C_code.pdf
 
 Library may be used freely and without limit with attribution.
 
*/
  
#include "MS5803.h"
#include "I2Cdev.h"

MS5803::MS5803(I2Cdev* i2c_bus)
{
 _i2c_bus = i2c_bus;
}


void MS5803::Reset()
{
 _i2c_bus->writeCommand(MS5803_ADDRESS, MS5803_RESET, true);
 }


uint32_t MS5803::DataRead(uint8_t CMD, uint8_t OSR)
{
        uint8_t data[3] = {0,0,0};
        _i2c_bus->writeCommand(MS5803_ADDRESS, CMD | OSR, false);
        
        switch (OSR)
        {
          case ADC_256: delay(1); break;         // delay for conversion to complete
          case ADC_512: delay(3); break;
          case ADC_1024: delay(4); break;
          case ADC_2048: delay(6); break;
          case ADC_4096: delay(10); break;
        }
     
         _i2c_bus->readBytes(MS5803_ADDRESS, MS5803_ADC_READ, 3, &data[0]);// Put read results in the Rx buffer
         return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
        }


void MS5803::PromRead(uint16_t * destination)
{
        uint8_t data[2] = {0,0};
        for (uint8_t ii = 0; ii < 8; ii++) 
        {
          _i2c_bus->readBytes(MS5803_ADDRESS, 0xA0 | ii << 1, 2, &data[0]);// Put read results in the Rx buffer
          destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
        }
}


unsigned char MS5803::checkCRC(uint16_t * n_prom)
{
    int cnt;
    unsigned int n_rem;
    unsigned int crc_read;
    unsigned char  n_bit;
    
    n_rem = 0x00;
    crc_read = n_prom[7];
    n_prom[7] = ( 0xFF00 & ( n_prom[7] ) );
    
    for (cnt = 0; cnt < 16; cnt++)
    { // choose LSB or MSB
        if ( cnt%2 == 1 ) n_rem ^= (unsigned short) ( ( n_prom[cnt>>1] ) & 0x00FF );
        else n_rem ^= (unsigned short) ( n_prom[cnt>>1] >> 8 );
        for ( n_bit = 8; n_bit > 0; n_bit-- )
        {
            if ( n_rem & ( 0x8000 ) )
            {
                n_rem = ( n_rem << 1 ) ^ 0x3000;
            }
            else {
                n_rem = ( n_rem << 1 );
            }
        }
    }
    
    n_rem = ( 0x000F & ( n_rem >> 12 ) );// // final 4-bit remainder is CRC code
    n_prom[7] = crc_read; // restore the crc_read to its original place
    
return ( n_rem ^ 0x00 ); // The calculated CRC should match what the device initally returned.
}

