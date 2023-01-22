The **GasCap Asset Tracker** is inspired by the GPS position tracker hidden in a car's gas cap featured in the *Breaking Bad* spin-off *[Better Call Saul](https://www.thewrap.com/better-call-saul-mike-just-explained/)*. The fictional device requires some kind of large military receiver and such a system (if it even existed) would likely cost 100x what the LoRaWAN-enabled GasCap Asset Tracker costs. 

"[Typical](https://www.walmart.com/ip/Stant-OEM-Replacement-Fuel-Cap-10834/44580281?athcpid=44580281&athpgid=AthenaItempage&athcgid=null&athznid=siext&athieid=v0&athstid=CS004&athguid=wWXGV-gRkWU2q6UUWRoStCm3JRDTDs7Vwa6c&athancid=null&athena=true)" gas cap shell outer dimensions (lots of variation here) are 2 5/8 inch wide by 1 5/8 inch deep (or ~67 mm x ~41 mm); I sized the device to fit within these dimensions while being powered by a single AA-sized 3.6 V, 2400 mAH LiSOCl2 battery. This would allow the device to remain in operation for weeks or months, depending on the GNSS duty cycle. 

While I don't seriously expect anyone to install one of these devices into a car gas cap, it will fit in some surprisingly tight spaces. And at just 7.1 g w/o battery, it is light enough to enable small animal tracking; at 16 mm x 62 mm, unobtrusive enough to enable bicycle tracking; and at less than $90, inexpensive enough to satisfy the requirements of many position tracking use cases. 

![GasCap.v02f](https://user-images.githubusercontent.com/6698410/213895220-9aa72419-d91f-4d79-871d-34e62ef75ffd.jpg)

Ultra-low-power, 16 mm x 62 mm asset tracker consisting of CMWX1ZZABZ-078 (SX1276 LoRa radio and STM32L082 host MCU), 
CAM M8Q concurrent GNSS module, MX25R6435 8 MByte SPI NOR flash memory for data logging, LIS2DW12 accelerometer for wake-on-motion/sleep-on-no-motion functionality, and BME280 for P/T/H environmental sensing. 

Use the **Cricket** board variant and Thomas Roell's superb [Arduino core](https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0) to program the GasCap via USB and the Arduino IDE.

I use the SPIFlash_Test sketch to erase the flash before data logging. One could also construct a circular buffer and block erase in a FIFO scheme. Simply erasing the whole flash and page writing every 256 bytes starting on page 1 is the simplest way and the one I have been using.

Use the readSPIFlash sketch to decode the logged data into comma-delimited CSV suitable for plotting in a spreadsheet.

Avalaible for sale on [Tindie](https://www.tindie.com/products/tleracorp/gascap-loragnss-asset-tracker/)!

Copyright 2022 Tlera Corporation
