Basic example sketch showing how to make use of the Grasshopper (STM32L082) development board as an I2C slave to a Dragonfly (STM32L476) host MCU.

Grasshopper I2C slave (I2C address 0x7C) is set to read its analog voltage and MCU temperature at 1 Hz, then send a data ready interrupt from slave pin 10 to the master pin 9. The Dragonfly master sets a newData flag in the interrupt handler, checks in the main loop for newData, then reads four bytes from the slave Grasshopper and recostructs the slave temperature and analog voltage, which is output on the serial monitor via master UASB serial.

This is probably the simplest example of an I2C slave. Missing is a register structure where configuration of the Grasshopper and sensor data could be stored. This would provide a more definite mechanism for the master to configure the data rate, query slave ID, and even send slave data via LoRaWAN, etc. 
