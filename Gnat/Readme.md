Ultra-low power asset tracker consisting of CMWX1ZZABZ SX1276 LoRa radio and STM32L082 host MCU, 
MAX M8Q concurrent GNSS module, BMA400 accelerometer for wake-on-motion/sleep-on-no-motion functionality. 

Molex PicoBlade connector exposes 3V3/GND/SDA/SCL as well as two digital pins (SWD/UART) and two analog pins for connecting daughter boards. 

Total sleep current is ~2.5 uA with BMA400 motion watchdog continuously monitoring state of motion. 
Total current with LoRaWAN updated every 10 minutes and GNSS updated every two hours is < 250 uA.

Full power control on the MAX M8Q with GPIO-enabled VDD through a dedicated 3.0 V LDO and a separate GPIO-enabled RTC backup. MAX M8Q may be
powered down with RTC backup to allow hot start fixes every four hours (ephemeris need updating every four hours) or less. 
The MAX M8Q RTC backup can also be powered off with zero current draw for cold fixes at longer-than-4-hour intervals.

Gnat is wired to support LoRa (FSK, GFSK, LoRa radio), LoRaWAN, and SigFox.

External whip (LoRa) and active patch (GNSS) antennas are required.

Board is intended to be powered by a 1S 3.7 V LiPo battery but any source between 3.3 and 5.5 V will do. 
LiSO2Cl batteries are ideal for long-term deployement.
