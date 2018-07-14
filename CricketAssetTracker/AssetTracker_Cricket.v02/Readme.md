Retro Cricket Asset Tracker sketch, variation on previous versions but tuned for highest GNSS position accuracy. 

Typical usage is read and log sensor data at 1 minute intervals, get GNSS fix every two hours but every minute when motion is detected and keep tracking until EPHE (estimated horizontal position error ) is less than 10 meter, and update all data via LoRaWAN every ten minutes.

At EPHE = 10, the Cricket will use ~550 uA, at EPHE = 20 the Cricket will use ~0.440 uA, and at EPHE = 50 the Cricket will use ~320 uA in this mode, depending on how often the device is in motion. The base current with all of the above but without the GNSS engine is ~40 uA, and the lowest power sleep current with everything off is ~14 uA.

If motion is expected less frequently, say once per day, then setting the long duty cycle to 12 or 24 hours will reduce the current usage proportionally; i.e., at 12 hours, the current usage should be ~(320 - 40) *2/12 + 40 ~ 90 uA, etc.

YMMV so test using a 105 or 150 mAH LiPo battery to see how many days the Cricket lasts with your specific duty cycle.

