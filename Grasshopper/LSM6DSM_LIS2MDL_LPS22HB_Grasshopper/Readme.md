Demonstration of Grasshopper use on a breadboard and running an [all-ST sensor breakout](https://www.tindie.com/products/onehorse/all-st-motion-sensor-breakout-board/) with the LSM6DSM accel/gyro, LIS2MDL magnetometer, and LPS22HB pressure/temperature sensor. The STM32L082 at 32 MHz runs the open-source Madgwick sensor fusion at 680 Hz, which is quite sufficient for ~3 degree heading accuracy with this fine sensor suite when properly calibrated.

This is a nice test of the interrupt handling capability of the STM32L082 since the three sensors each have a data ready interrupt (the LSM6DSM has one for the accel and one for the gyro, or one for data ready and one for motion detect, etc.) being used here.

![GrasshopperBreadboard](https://user-images.githubusercontent.com/6698410/34959692-7793e4ea-f9ec-11e7-8213-1dd3e37eee73.jpg)
