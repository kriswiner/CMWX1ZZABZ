/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The BMA280 is an inexpensive (~$1), three-axis, high-resolution (14-bit) acclerometer in a tiny 2 mm x 2 mm LGA12 package with 32-slot FIFO, 
 *  two multifunction interrupts and widely configurable sample rate (15 - 2000 Hz), full range (2 - 16 g), low power modes, 
 *  and interrupt detection behaviors. This accelerometer is nice choice for low-frequency sound and vibration analysis,
 *  tap detection and simple orientation estimation.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
 
#ifndef BMA280_h
#define BMA280_h

#include "Arduino.h"
#include <Wire.h>

/* Register Map BMA280
// http://www.mouser.com/ds/2/783/BST-BMA280-DS000-11_published-786496.pdf
*/
#define BMA280_BGW_CHIPID      0x00
#define BMA280_ACCD_X_LSB      0x02
#define BMA280_ACCD_X_MSB      0x03
#define BMA280_ACCD_Y_LSB      0x04
#define BMA280_ACCD_Y_MSB      0x05
#define BMA280_ACCD_Z_LSB      0x06
#define BMA280_ACCD_Z_MSB      0x07
#define BMA280_ACCD_TEMP       0x08
#define BMA280_INT_STATUS_0    0x09
#define BMA280_INT_STATUS_1    0x0A
#define BMA280_INT_STATUS_2    0x0B
#define BMA280_INT_STATUS_3    0x0C
#define BMA280_FIFO_STATUS     0x0E
#define BMA280_PMU_RANGE       0x0F
#define BMA280_PMU_BW          0x10
#define BMA280_PMU_LPW         0x11
#define BMA280_PMU_LOW_POWER   0x12
#define BMA280_ACCD_HBW        0x13
#define BMA280_BGW_SOFTRESET   0x14
#define BMA280_INT_EN_0        0x16
#define BMA280_INT_EN_1        0x17
#define BMA280_INT_EN_2        0x18
#define BMA280_INT_MAP_0       0x19
#define BMA280_INT_MAP_1       0x1A
#define BMA280_INT_MAP_2       0x1B
#define BMA280_INT_SRC         0x1E
#define BMA280_INT_OUT_CTRL    0x20
#define BMA280_INT_RST_LATCH   0x21
#define BMA280_INT_0           0x22
#define BMA280_INT_1           0x23
#define BMA280_INT_2           0x24
#define BMA280_INT_3           0x25
#define BMA280_INT_4           0x26
#define BMA280_INT_5           0x27
#define BMA280_INT_6           0x28
#define BMA280_INT_7           0x29
#define BMA280_INT_8           0x2A
#define BMA280_INT_9           0x2B
#define BMA280_INT_A           0x2C
#define BMA280_INT_B           0x2D
#define BMA280_INT_C           0x2E
#define BMA280_INT_D           0x2F
#define BMA280_FIFO_CONFIG_0   0x30
#define BMA280_PMU_SELF_TEST   0x32
#define BMA280_TRIM_NVM_CTRL   0x33
#define BMA280_BGW_SPI3_WDT    0x34
#define BMA280_OFC_CTRL        0x36
#define BMA280_OFC_SETTING     0x37
#define BMA280_OFC_OFFSET_X    0x38
#define BMA280_OFC_OFFSET_Y    0x39
#define BMA280_OFC_OFFSET_Z    0x3A
#define BMA280_TRIM_GP0        0x3B
#define BMA280_TRIM_GP1        0x3C
#define BMA280_FIFO_CONFIG_1   0x3E
#define BMA280_FIFO_DATA       0x3F

#define BMA280_ADDRESS  0x18  // if ADO is 0 (default)

#define AFS_2G  0x03
#define AFS_4G  0x05
#define AFS_8G  0x08
#define AFS_16G 0x0C

#define BW_7_81Hz  0x08  // 15.62 Hz sample rate, etc
#define BW_15_63Hz 0x09
#define BW_31_25Hz 0x0A
#define BW_62_5Hz  0x0B
#define BW_125Hz   0x0C  // 250 Hz sample rate
#define BW_250Hz   0x0D
#define BW_500Hz   0x0E
#define BW_1000Hz  0x0F  // 2 kHz sample rate == unfiltered data

#define normal_Mode      0x00  //define power modes
#define deepSuspend_Mode 0x01
#define lowPower_Mode    0x02
#define suspend_Mode     0x04

#define lp_mode_1        0x00 // two types of low power mode
#define lp_mode_2        0x40

#define sleep_0_5ms   0x05  // define sleep duration in low power modes
#define sleep_1ms     0x06
#define sleep_2ms     0x07
#define sleep_4ms     0x08
#define sleep_6ms     0x09
#define sleep_10ms    0x0A
#define sleep_25ms    0x0B
#define sleep_50ms    0x0C
#define sleep_100ms   0x0D
#define sleep_500ms   0x0E
#define sleep_1000ms  0x0F

class BMA280
{
  public: 
  BMA280(uint8_t intPin1, uint8_t intPin2);
  float getAres(uint8_t Ascale);
  uint8_t getChipID();
  uint8_t getTapType();
  uint8_t getTapStatus();
  void initBMA280(uint8_t Ascale, uint8_t BW, uint8_t power_Mode, uint8_t sleep_dur);
  void initBMA280_MotionManager(uint8_t Ascale, uint8_t BW, uint8_t power_Mode,
                                          uint8_t sleep_dur, uint8_t low_power_Mode,
                                          uint8_t motion_threshold);
  void           activateNoMotionInterrupt();
  void           deactivateNoMotionInterrupt();
  void           activateSlopeInterrupt();
  void           deactivateSlopeInterrupt();
  void fastCompensationBMA280();
  void resetBMA280();
  void selfTestBMA280();
  void readBMA280AccelData(int16_t * destination);
  int16_t readBMA280TempData();
  void I2Cscan();
  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t readByte(uint8_t address, uint8_t subAddress);
  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
  private:
  uint8_t _intPin1;
  uint8_t _intPin2;
  float _aRes;
};

#endif
