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
 
#ifndef LIS2DW12_h
#define LIS2DW12_h

#include "Arduino.h"
#include "I2CDev.h"
#include <Wire.h>

/* Register Map LIS2DW12
// https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-LIS2DW12-DS000-02.pdf
*/
#define LIS2DW12_OUT_T_L                  0x0D
#define LIS2DW12_OUT_T_H                  0x0E
#define LIS2DW12_WHO_AM_I                 0x0F  // should be 0x44
#define LIS2DW12_CTRL1                    0x20
#define LIS2DW12_CTRL2                    0x21
#define LIS2DW12_CTRL3                    0x22
#define LIS2DW12_CTRL4_INT1_PAD_CTRL      0x23
#define LIS2DW12_CTRL5_INT2_PAD_CTRL      0x24
#define LIS2DW12_CTRL6                    0x25
#define LIS2DW12_OUT_T                    0x26
#define LIS2DW12_STATUS                   0x27
#define LIS2DW12_OUT_X_L                  0x28
#define LIS2DW12_OUT_X_H                  0x29
#define LIS2DW12_OUT_Y_L                  0x2A
#define LIS2DW12_OUT_Y_H                  0x2B
#define LIS2DW12_OUT_Z_L                  0x2C
#define LIS2DW12_OUT_Z_H                  0x2D
#define LIS2DW12_FIFO_CTRL                0x2E
#define LIS2DW12_FIFO_SAMPLES             0x2F
#define LIS2DW12_TAP_THS_X                0x30
#define LIS2DW12_TAP_THS_Y                0x31
#define LIS2DW12_TAP_THS_Z                0x32
#define LIS2DW12_INT_DUR                  0x33
#define LIS2DW12_WAKE_UP_THS              0x34
#define LIS2DW12_WAKE_UP_DUR              0x35
#define LIS2DW12_FREE_FALL                0x36
#define LIS2DW12_STATUS_DUP               0x37
#define LIS2DW12_WAKE_UP_SRC              0x38
#define LIS2DW12_TAP_SRC                  0x39
#define LIS2DW12_SIXD_SRC                 0x3A
#define LIS2DW12_ALL_INT_SRC              0x3B
#define LIS2DW12_X_OFS_USR                0x3C
#define LIS2DW12_Y_OFS_USR                0x3D
#define LIS2DW12_Z_OFS_USR                0x3E
#define LIS2DW12_CTRL_REG7                0x3F


#define LIS2DW12_ADDRESS  0x19  // if ADO is 1 (default since internally pulled up by ~30 kOhm resistor)
// #define LIS2DW12_ADDRESS  0x18  // if ADO is pulled to GND

typedef enum {
  LIS2DW12_LP_MODE_1                = 0x00,
  LIS2DW12_LP_MODE_2                = 0x01,
  LIS2DW12_LP_MODE_3                = 0x02,
  LIS2DW12_LP_MODE_4                = 0x03
} LPMODE;


typedef enum  {
  LIS2DW12_MODE_LOW_POWER           = 0x00,
  LIS2DW12_MODE_HIGH_PERF           = 0x01,
  LIS2DW12_MODE_SINGLE_CONV         = 0x02
} MODE;


typedef enum {
  LIS2DW12_ODR_POWER_DOWN           = 0x00,
  LIS2DW12_ODR_12_5_1_6HZ           = 0x01,
  LIS2DW12_ODR_12_5Hz               = 0x02,
  LIS2DW12_ODR_25Hz                 = 0x03,
  LIS2DW12_ODR_50Hz                 = 0x04,
  LIS2DW12_ODR_100Hz                = 0x05,
  LIS2DW12_ODR_200Hz                = 0x06,
  LIS2DW12_ODR_400_200Hz            = 0x07,
  LIS2DW12_ODR_800_200Hz            = 0x08,
  LIS2DW12_ODR_1600_200Hz           = 0x09
} ODR;


typedef enum {
  LIS2DW12_FS_2G                    = 0x00,
  LIS2DW12_FS_4G                    = 0x01,
  LIS2DW12_FS_8G                    = 0x02,
  LIS2DW12_FS_16G                   = 0x03
} FS;


typedef enum  {
  LIS2DW12_BW_FILT_ODR2             = 0x00,
  LIS2DW12_BW_FILT_ODR4             = 0x01,
  LIS2DW12_BW_FILT_ODR10            = 0x02,
  LIS2DW12_BW_FILT_ODR20            = 0x03
} BW_FILT;


typedef enum  {
  BYPASS                            = 0x00,
  FIFO                              = 0x01,
  CONT_TO_FIFO                      = 0x03,
  BYPASS_TO_CONT                    = 0x04,
  CONTINUOUS                        = 0x06
} FIFOMODE;



class LIS2DW12
{
  public: 
  LIS2DW12(I2Cdev* i2c_bus);
  uint8_t getChipID();
  void init(uint8_t fs, uint8_t odr, uint8_t mode, uint8_t lpMode, uint8_t bw, bool lowNoise);
  void Compensation(uint8_t fs, uint8_t odr, uint8_t mode, uint8_t lpMode, uint8_t bw, bool lowNoise, float * offset);
  void reset();
  void selfTest(float * destination);
  void readAccelData(int16_t * destination);
  int16_t readTempData();
  uint8_t readRawTempData();
  void activateNoMotionInterrupt();
  void deactivateNoMotionInterrupt();
  uint8_t getStatus();
  void powerDown();
  void powerUp(uint8_t odr);
  uint8_t getWakeSource();
  void configureFIFO(uint8_t FIFOMode, uint8_t FIFOThreshold);
  uint8_t FIFOsamples();

  private:
  float _aRes;
  I2Cdev* _i2c_bus;
};

#endif
