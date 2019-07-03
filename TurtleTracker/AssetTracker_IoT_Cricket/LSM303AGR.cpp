/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LSM303AGR is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "LSM303AGR.h"
#include "I2CDev.h"

LSM303AGR::LSM303AGR(I2Cdev* i2c_bus)
{
 _i2c_bus = i2c_bus;   
}


uint8_t LSM303AGR::getChipID()
{
  uint8_t c = _i2c_bus->readByte(LSM303AGR_ADDRESS, LSM303AGR_WHO_AM_I);
  return c;
}


void LSM303AGR::reset()
{
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG5_A, 0x80);
}


float LSM303AGR::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    case AFS_2G:
 //        _aRes = 2.0f/512.0f; //normal mode
         _aRes = 2.0f/2048.0f; // hi-res mode
         return _aRes;
         break;
    case AFS_4G:
 //        _aRes = 4.0f/512.0f; // normal mode
         _aRes = 4.0f/2048.0f; // high-res mode
         return _aRes;
         break;
    case AFS_8G:
//         _aRes = 8.0f/512.0f; // normal mode
         _aRes = 8.0f/2048.0f; // hi-res mode
         return _aRes;
         break;
    case AFS_16G:
//         _aRes = 16.0f/512.0f; // normal mode
          _aRes = 16.0f/2048.0f; //hi-res mode
        return _aRes;
         break;
  }
}



void LSM303AGR::init(uint8_t Ascale, uint8_t AODR)
{
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_TMP_CFG_REG_A, 0xC0); // enable accelerometer temperature sensor
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG1_A, AODR << 4 | 0x07); // normal mode, enable all axes
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG3_A, 0x10); // data ready on INT1
//  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG4_A, 0x80 | Ascale << 4); // BDU, full scale range, normal mode
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG4_A, 0x80 | 0x08 | Ascale << 4); // BDU, full scale range, high-res mode
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG6_A, 0x08); // activity interrupt on INT2 

  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_ACT_THS_A, 0x0A); // set no-motion threshold to 10 x 2 g/128 mg ~ 150 mg
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_ACT_DUR_A, 0x40); // set no-motion duration to 64 x 8/100 Hz ~ 5 sec
}


void LSM303AGR::selfTest()
{
  int16_t temp[3] = {0, 0, 0};
  int16_t accelTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0};

  // initialize accel for self test
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG2_A, 0x00);       
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG3_A, 0x00);       
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG4_A, 0x80);       
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG1_A, 0x57); 
  delay(100);      

  bool status = false;
  while(status == false) {
    status = (_i2c_bus->readByte(LSM303AGR_ADDRESS, LSM303AGR_STATUS_REG_A)) & 0x08; // wait until all axes have new data
  }
  readAccData(temp); // read and discard data

  for (uint8_t i = 0; i < 5; i++){
  readAccData(temp);
  accelNom[0] = temp[0];
  accelNom[1] = temp[1];
  accelNom[2] = temp[2];
    }
  accelNom[0] /= 5.0f; // average data
  accelNom[1] /= 5.0f;
  accelNom[2] /= 5.0f;
  
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG4_A, 0x84); // enable accel self test
  delay(100); // let accel respond

  status = false;
  while(status == false) {
    status = (_i2c_bus->readByte(LSM303AGR_ADDRESS, LSM303AGR_STATUS_REG_A)) & 0x08; // wait until all axes have new data
  }
  readAccData(temp); // read and discard data
  
  for (uint8_t i = 0; i < 5; i++){
  readAccData(temp);
  accelTest[0] = temp[0];
  accelTest[1] = temp[1];
  accelTest[2] = temp[2];
  }
  accelTest[0] /= 5.0f; // average data
  accelTest[1] /= 5.0f;
  accelTest[2] /= 5.0f;

  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG1_A, 0x00); 
  _i2c_bus->writeByte(LSM303AGR_ADDRESS, LSM303AGR_CTRL_REG4_A, 0x80);
  delay(100); // let accel respond

  Serial.println("Accel Self Test:");
  Serial.print("Ax results:"); Serial.print(  (accelTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("Ay results:"); Serial.println((accelTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("Az results:"); Serial.println((accelTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 17 and 360 mg");
  delay(2000);
}


void LSM303AGR::offsetBias(float * dest)
{
  int16_t temp[3] = {0, 0, 0};
  int32_t sum[3] = {0, 0, 0};
    
  Serial.println("Calculate accel offset biases: keep sensor flat and motionless!");
  delay(4000);

  for (int ii = 0; ii < 128; ii++)
  {
    readAccData(temp);
    sum[0] += temp[0];
    sum[1] += temp[1];
    sum[2] += temp[2];
    delay(50);
  }

  dest[0] = (float) sum[0]*_aRes/128.0f;
  dest[1] = (float) sum[1]*_aRes/128.0f;
  dest[2] = (float) sum[2]*_aRes/128.0f;

  if(dest[0] > 0.8f)  {dest[0] -= 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest[0] < -0.8f) {dest[0] += 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest[1] > 0.8f)  {dest[1] -= 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest[1] < -0.8f) {dest[1] += 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest[2] > 0.8f)  {dest[2] -= 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
  if(dest[2] < -0.8f) {dest[2] += 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
}


void LSM303AGR::readAccData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(LSM303AGR_ADDRESS, 0x80 | LSM303AGR_OUT_X_L_A, 6, &rawData[0]);  // Read the 6 raw data registers into data array
// normal mode
//  destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]) >> 6;  // Turn the MSB and LSB into a signed 10-bit value in normal mode
//  destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) >> 6;  
//  destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) >> 6; 
// hi-res mode
  destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value in normal mode
  destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) >> 4;  
  destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) >> 4; 
}


int16_t LSM303AGR::readAccTempData()
{
  uint8_t rawData[2];  // high/low temperature data stored here
  _i2c_bus->readBytes(LSM303AGR_ADDRESS, 0x80 | LSM303AGR_OUT_TEMP_L_A, 2, &rawData[0]);  // Read the 2 raw data registers into data array
  return (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]) >> 8;  // Turn the MSB and LSB into a signed 8-bit value  
}

