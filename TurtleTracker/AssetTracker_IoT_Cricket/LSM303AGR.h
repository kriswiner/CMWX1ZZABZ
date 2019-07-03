/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LSM303AGR is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef LSM303AGR_h
#define LSM303AGR_h

#include "Arduino.h"
#include <Wire.h>
#include "I2CDev.h"

//Register map for LSM303AGR'
//https://www.st.com/content/ccc/resource/technical/document/datasheet/74/c4/19/54/62/c5/46/13/DM00177685.pdf/files/DM00177685.pdf/jcr:content/translations/en.DM00177685.pdf

#define LSM303AGR_STATUS_REG_AUX_A      0x07
#define LSM303AGR_OUT_TEMP_L_A          0x0C
#define LSM303AGR_OUT_TEMP_H_A          0x0D
#define LSM303AGR_INT_COUNTER_REG_A     0x0E
#define LSM303AGR_WHO_AM_I              0x0F  // should return 0x33
#define LSM303AGR_TMP_CFG_REG_A         0x1F
#define LSM303AGR_CTRL_REG1_A           0x20
#define LSM303AGR_CTRL_REG2_A           0x21
#define LSM303AGR_CTRL_REG3_A           0x22
#define LSM303AGR_CTRL_REG4_A           0x23
#define LSM303AGR_CTRL_REG5_A           0x24
#define LSM303AGR_CTRL_REG6_A           0x25
#define LSM303AGR_REF_DATACAPTURE_A     0x26
#define LSM303AGR_STATUS_REG_A          0x27
#define LSM303AGR_OUT_X_L_A             0x28
#define LSM303AGR_OUT_X_H_A             0x29
#define LSM303AGR_OUT_Y_L_A             0x2A
#define LSM303AGR_OUT_Y_H_A             0x2B
#define LSM303AGR_OUT_Z_L_A             0x2C
#define LSM303AGR_OUT_Z_H_A             0x2D
#define LSM303AGR_FIFO_CTRL_REG_A       0x2E
#define LSM303AGR_FIFO_SRC_REG_A        0x2F
#define LSM303AGR_INT1_CFG_A            0x30
#define LSM303AGR_INT1_SRC_A            0x31
#define LSM303AGR_INT1_THS_A            0x32
#define LSM303AGR_INT1_DUR_A            0x33
#define LSM303AGR_INT2_CFG_A            0x34
#define LSM303AGR_INT2_SRC_A            0x35
#define LSM303AGR_INT2_THS_A            0x36
#define LSM303AGR_INT2_DUR_A            0x37
#define LSM303AGR_CLICK_CFG_A           0x38
#define LSM303AGR_CLICK_SRC_A           0x39
#define LSM303AGR_CLICK_THS_A           0x3A
#define LSM303AGR_TIME_LIMIT_A          0x3B
#define LSM303AGR_TIME_LATENCY_A        0x3C
#define LSM303AGR_TIME_WNDOW_A          0x3D
#define LSM303AGR_ACT_THS_A             0x3E
#define LSM303AGR_ACT_DUR_A             0x3F

#define LSM303AGR_ADDRESS           0x19  // Address of LSM303AGR accel/gyro when ADO = 0

#define AFS_2G  0x00
#define AFS_4G  0x01
#define AFS_8G  0x02
#define AFS_16G 0x03

#define AODR_1Hz     0x01   
#define AODR_10Hz    0x02
#define AODR_25Hz    0x03
#define AODR_50Hz    0x04
#define AODR_100Hz   0x05
#define AODR_200Hz   0x06
#define AODR_400Hz   0x07
#define AODR_1620Hz  0x08 // low power mode only
#define AODR_5376Hz  0x09 // low power mode only
#define AODR_1344Hz  0x09 // normal/ HR power mode only

class LSM303AGR
{
  public:
  LSM303AGR(I2Cdev* i2c_bus);
  float getAres(uint8_t Ascale);
  void reset();
  uint8_t getChipID();
  void init(uint8_t Ascale, uint8_t AODR);
  void offsetBias(float * dest);
  void selfTest();
  void readAccData(int16_t * destination);
  int16_t readAccTempData();
  private:
  float _aRes;
  I2Cdev* _i2c_bus;
};

#endif
