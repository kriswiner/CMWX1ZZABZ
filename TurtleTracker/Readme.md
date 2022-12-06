**Indian Ocean sea Turtle (IoT) Tracker**

See [here](https://ocean-indien.ifremer.fr/en/Projects/Technological-innovations/pIOT-2018-2020-IOT-2018-2021/IOT-2018-2021) and [here](https://archimer.ifremer.fr/doc/00773/88525/94245.pdf) for details.

Based on the CMWX1ZZABZ LoRaWAN module (STM32L082 Cortex M0+ MCU + SX1276 LoRa Radio modem) and using UBLOX's MAX M8Q GNSS engine this 20.7 x 37.5 mm animal tracker
embeds two barometers (MS5803-01BA and MS5837-30BA) for underwater depth measurement, a VEML6030 ambient light sensor, and a combination accelerometer/magnetometer 
(LSM303AGR e-compass) for simple heading mearurements. The LoRaWAN chip/pcb antenna is intended to serve as backup for an external LoRaWAN antenna. 
The device has an 8 MByte SPI NOR flash memory for data logging, a 500 mA  (STBC08) LiPo battery charger, a dual MOSFET for battery voltage measurement,
and an indicator led. There is a resistance circuit for an external seawater immersion detector. The device is designed to use very little power with 
components chosen for their low quiescent current; the CMWX1ZZABZ has sleep current of 1.65 uA. All sensors and GNSS engine can be switched off so the
power usage can be configured to match the particular conditions/application. The goal is a one year battery life using a small LiPo battery.

![IoT Turtle Tracker top](https://user-images.githubusercontent.com/6698410/60612972-c8b11780-9d7e-11e9-9736-92715d28e852.jpg)
![IoT Turtle Tracker bottom](https://user-images.githubusercontent.com/6698410/60612987-d23a7f80-9d7e-11e9-9c5e-ad036ecf305e.jpg)
