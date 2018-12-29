Ultra-low power 20 mm x 20 mm asset tracker consisting of CMWX1ZZABZ (SX1276 LoRa radio and STM32L082 host MCU), 
MAX M8Q concurrent GNSS module, BMA400 accelerometer for wake-on-motion/sleep-on-no-motion functionality. 

![GnatTop](https://user-images.githubusercontent.com/6698410/47467809-f637d100-d7ac-11e8-96e4-18dce376081b.jpg)
![GnatBottom](https://user-images.githubusercontent.com/6698410/47467808-f506a400-d7ac-11e8-9fa0-d7acf583e157.jpg)

Molex PicoBlade connector exposes 3V3/GND/SDA/SCL as well as two digital pins (SWD/UART) and two analog pins for connecting daughter boards. 

Total sleep current is 2.5 uA with BMA400 motion watchdog continuously monitoring state of motion. 
Total current with LoRaWAN updated every 10 minutes and GNSS updated every two hours is < 250 uA.

Full power control on the MAX M8Q with GPIO-enabled VDD through a dedicated 3.0 V LDO and a separate GPIO-enabled RTC backup. MAX M8Q may be
powered down with RTC backup to allow hot start fixes every four hours (when ephemeris expires) or less. 
The MAX M8Q RTC backup can also be powered off with zero current draw for cold fixes at longer-than-4-hour intervals.

Gnat is wired to support LoRa (FSK, GFSK, LoRa radio), LoRaWAN, and SigFox.

External [whip](https://www.mouser.com/ProductDetail/Anaren/66089-0930?qs=pH7abCSN9NM0tHwbfikfEQ%3d%3d) (LoRa) and active [patch](https://www.mouser.com/ProductDetail/Inventek/ACTPAT184-01-IP?qs=sGAEpiMZZMuBTKBKvsBmlN73K%2f2BcYXlCZkEPYT616pzDXusmnq0TA%3d%3d) (GNSS) antennas are required.

Board is intended to be powered by a 1S 3.7 V LiPo battery but any source between 3.6 and 5.5 V will do. 
[LiSOCl2](https://www.eemb.com/battery/primary-battery/li-socl2-battery.html) batteries are ideal for long-term deployment. A 2600 mAH 3.6 V LiSOCl2 AA-sized battery @ 1 location fix per day will last more than ten years.

Programmable via the USB connector using the Arduino IDE, of course!
