/* 9/18/21 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer   
 *  
 *  The LIS2DW12 is an inexpensive (~$1), three-axis, medium-resolution (12- or 14-bit), ultra-low power 
 *  (<1 uA low power mode) accelerometer in a tiny 2 mm x 2 mm LGA12 package with a 192-byte FIFO, 
 *  two multifunction interrupts and widely configurable sample rate (1.6 - 1600 Hz), full range (2 - 16 g), 
 *  low power modes, and interrupt detection behaviors. This accelerometer is nice choice for motion-based 
 *  wake/sleep, tap detection, step counting, and simple orientation estimation.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include "LIS2DW12.h"
#include "I2CDev.h"

LIS2DW12::LIS2DW12(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t LIS2DW12::getChipID()
{
  uint8_t c = _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_WHO_AM_I);
  return c;
}

uint8_t LIS2DW12::getStatus()
{
  uint8_t c = _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_STATUS);
  return c;
}


void LIS2DW12::init(uint8_t fs, uint8_t odr, uint8_t mode, uint8_t lpMode, uint8_t bw, bool lowNoise)
{
   // Normal mode configuration //
   // sample rate (bits 4 - 7), power mode (bits 2-3), and low-power mode (bits 0-1)
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, odr << 4 | mode << 2 | lpMode); 
   // bandwidth bits (6-7), full-scale range bit (4-5)
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL6, bw << 6 | fs << 4);           
   if(lowNoise)   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL6, bw << 6 | fs << 4 | 0x04);   // set low noise bit 2        

   _aRes = 0.000244f * (1 << fs);                                       // scale resolutions per LSB for the sensor at 14-bit data 

   // enable block data update (bit 3) and auto register address increment (bit 2)
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, 0x08 | 0x04);    

   // enable latch interrupt for activity/no activity interrupts
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x00); // push pull, active high   

   // wake up (Bit 5) routed to interrupt 1
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x20); 
   // sleep change state (bit 6) routed to interrupt 2
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL5_INT2_PAD_CTRL, 0x40); 
   
   // enable sleep detect (bit 6), set wake threshold 1 LSB = 1/64 of full scale
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_WAKE_UP_THS, 0x40 | 0x02); //  62.5 mg threshold for wake on any axis, n x 31.25 mgs at 2 g FS
   // wake-up from sleep duration (bits 5-6) * n/odr, at 1.6 Hz max is 4/1.6 = 2.5 s, set to 0.625 s
   // inactivity delay before sleep (bits 0 - 3) 512*n/odr, at 25 Hz, 0 is 16/odr = 10s @ 1.6Hz, 1 is 320s @ 1.6 Hz 
   // set stationary bit 4
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_WAKE_UP_DUR, 0x20 | 0x10 | 0x00);  // set inactivity delay before sleep to 10s at 1.6 Hz odr
   // should limit in-motion GNSS data rate to once per 320s

   // pulse interrupt (bit 7), enable interrupt (bit 5) 
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL_REG7, 0x80 | 0x20);    
}


void LIS2DW12::activateNoMotionInterrupt()
{
  _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL5_INT2_PAD_CTRL,  0x40);           // enable GEN1 (no_Motion) interrupt  
}


void LIS2DW12::deactivateNoMotionInterrupt()
{
  _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL5_INT2_PAD_CTRL,  0x00);           // disable GEN1 (no_Motion) interrupt  
}


  void LIS2DW12::Compensation(uint8_t fs, uint8_t odr, uint8_t mode, uint8_t lpMode, uint8_t bw, bool lowNoise, float * offset)
  {     
    int16_t temp[3] = {0, 0, 0};
    int32_t sum[3] = {0, 0, 0};

    _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, 0x08 | 0x04); // Block update and auto increment registers 
    _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x00);    
    _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x00); 
    _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL5_INT2_PAD_CTRL, 0x00); 
    // bandwidth bits (6-7), full-scale range bit (4-5)
    _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL6, bw << 6 | fs << 4 );         
    if(lowNoise) _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL6, bw << 6 | fs << 4 | 0x04);   // set low noise bit 2        
    // sample rate (bits 4 - 7), power mode (bits 2-3), and low-power mode (bits 0-1)
    _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, odr << 4 | mode << 2 | lpMode ); 

    _aRes = 0.000244f * (1 << fs);                                       // scale resolutions per LSB for the sensor  
    delay(100);
    while( !( getStatus() & 0x01) ) { } // wait for data ready bit
    readAccelData(temp);                // read and discard data

    for(uint8_t ii = 0; ii < 32; ii++)
    {
       while( !(getStatus() & 0x01) ) { } // wait for data ready bit
       readAccelData(temp);
       sum[0] += (int32_t)temp[0];
       sum[1] += (int32_t)temp[1];
       sum[2] += (int32_t)temp[2];
    }
     
     offset[0] = float(sum[0])/32.0f;
     offset[1] = float(sum[1])/32.0f;
     offset[2] = float(sum[2])/32.0f;
     offset[0] *= _aRes;
     offset[1] *= _aRes;
     offset[2] *= _aRes;
     if(offset[2] > +0.5f) offset[2] = offset[2] - 1.0f;
     if(offset[2] < -0.5f) offset[2] = offset[2] + 1.0f;
     } /* end of accel calibration */


   void LIS2DW12::reset()
   {
    uint8_t temp = _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2);  
    _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, temp | 0x40); // software reset the LIS2DW12
   }


   void LIS2DW12::selfTest(float * destination)
   {
   int16_t temp[3]={0, 0, 0};
   // 5x sum of 13-bit (except sign) could be as much as 15.25 bits and too big for int16_t, but in 
   // practice nominal value s are < 1000 on 4 g scale and selfTest values are < 3000 so int16_t OK
   int16_t posX=0, posY=0, posZ=0, nomX=0, nomY=0, nomZ=0; // 5x sum of 13-bit (except sign) could be
   
   // initialize sensor for self test
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, 0x08 | 0x04);    
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x00);    
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x00); 
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL5_INT2_PAD_CTRL, 0x00); 
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL6, 0x10);         // set +/- 4g FS, LP filter ODR/2
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, 0x40 | 0x04 ); // set sample rate to 200 Hz, high performance mode

   float STres = 0.488f; // mg/LSB for 4 g full scale, high performance mode
   delay(100);
   while( !( getStatus() & 0x01) ) { } // wait for data ready bit
   readAccelData(temp);                // read and discard data
   
   // nominal axes test
   for (uint8_t ii = 0; ii < 5; ii++){
    while( !(getStatus() & 0x01) ) { } // wait for data ready bit
    readAccelData(temp);
    nomX += temp[0];
    nomY += temp[1];
    nomZ += temp[2];
   }
    
   // positive axes test
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x40); // positive axes
   delay(100);
   while( !(getStatus() & 0x01) ) { } // wait for data ready bit
   readAccelData(temp);               // read and discard data
   
   for (uint8_t ii = 0; ii < 5; ii++){
    while( !(getStatus() & 0x01) ) { } // wait for data ready bit
    readAccelData(temp);
    posX += temp[0];
    posY += temp[1];
    posZ += temp[2];
    }
   
   destination[0] = (float)(posX - nomX)*STres/5.0f;
   destination[1] = (float)(posY - nomY)*STres/5.0f;
   destination[2] = (float)(posZ - nomZ)*STres/5.0f;

   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, 0x00); // disable sensor
   _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x00); // disable self test
/* end of self test*/
}


  void LIS2DW12::readAccelData(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(LIS2DW12_ADDRESS, LIS2DW12_OUT_X_L, 6, &rawData[0]);  // Read the 6 raw data registers into data array
  destination[0] = ( (int16_t) ( (int16_t)rawData[1] << 8 ) | rawData[0]) >> 2;     // Turn the MSB and LSB into a signed 14-bit value
  destination[1] = ( (int16_t) ( (int16_t)rawData[3] << 8 ) | rawData[2]) >> 2;  
  destination[2] = ( (int16_t) ( (int16_t)rawData[5] << 8 ) | rawData[4]) >> 2; 
  }


  int16_t LIS2DW12::readTempData() {
  uint8_t temp = _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_OUT_T);    // Read the raw data register  
  int16_t tmp = (int16_t) ( ((int16_t)temp << 8) | 0x00) >> 8;  // Turn into signed 8-bit temperature value
  return tmp;
  }


  uint8_t LIS2DW12::readRawTempData() {
  uint8_t temp = _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_OUT_T);    // Read the raw data register  
  return temp;
  }


  void LIS2DW12::powerDown() {
  uint8_t temp = _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1);    // Read the raw data register  
  _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, temp & 0x0F );    // set odr to 0 (bits 4 - 7)
  }


  void LIS2DW12::powerUp(uint8_t odr) {
  uint8_t temp = _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1);    // Read the raw data register  
  _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, temp | odr << 4); // set odr (bits 4 - 7)
  }


  uint8_t LIS2DW12::getWakeSource( ) {
  uint8_t temp = _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_WAKE_UP_SRC);    // Read wake source register  
  return temp;
  }


  void LIS2DW12::configureFIFO(uint8_t FIFOMode, uint8_t FIFOThreshold) {
  _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_FIFO_CTRL, FIFOMode << 5 | FIFOThreshold );     
  }


  uint8_t LIS2DW12::FIFOsamples( ) {
  uint8_t temp = _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_FIFO_SAMPLES);    // Read FIFO samples register  
  return temp;
  }
  
