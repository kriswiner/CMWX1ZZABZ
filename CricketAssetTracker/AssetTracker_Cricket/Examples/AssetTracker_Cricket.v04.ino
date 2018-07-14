/* Cricket Asset Tracker contains:
   CMWX1ZZABZ (STM32L082 and SX1276)
   BMA280 accelerometer, BME280 pressure/humidity/temperature sensors
   CAM M8Q Concurrent GNSS engine
   MX25R6435FZAI 8 MByte SPI NOR flash memory
   https://hackaday.io/project/25790-asset-tracker
   Idea is ultra-low power for longest LiPo battery life so I would run this with
   4.2 MHz clock speed; this reduction plus use of STM32 stop mode means no serial
   through the USB.  
    This example code is in the public domain.
*/
#include <STM32L0.h>
#include "LoRaWAN.h"
#include "TimerMillis.h"
#include "GNSS.h"
#include <RTC.h>
#include "BMA280.h"
#include "BME280.h"
#include "SPIFlash.h"
#include "CayenneLPP.h"

// Cricket Asset Tracker cricketnode1
const char *appEui = "70B3D57ED000964D";
const char *appKey = "7DE66B18F7105B19A1427AFEB2514597";
const char *devEui = "9473730323239372";


CayenneLPP myLPP(64);

// Cricket pin assignments
#define myLed   10 // blue led 

TimerMillis LoRaTimer;

GNSSLocation myLocation;
GNSSSatellites mySatellites;

volatile bool isTracking = false;

TimerMillis NoMotionActivityTimer;  // instantiate low-frequency timer
TimerMillis InMotionActivityTimer;  // instantiate high-frequency timer

uint32_t UID[3] = {0, 0, 0}; 
char buffer[32];

bool SerialDebug = true;

// CAM M8Q GNSS configuration
#define pps      4 // 1 Hz fix pulse

uint8_t Hour = 12, Minute = 0, Second = 0, Millisec, Year = 1, Month = 1, Day = 1;
uint8_t hours = 12, minutes = 0, seconds = 0, year = 1, month = 1, day = 1;
uint32_t subSeconds, milliseconds;
bool ppsFlag = false, firstSync = false, alarmFlag = true;
uint16_t count = 0, fixType = 0, fixQuality, latBytes[4], longBytes[4], tempBytes[4], pressBytes[4];
int32_t latOut, longOut;
float Temperature, Long, Lat, Alt, EPE;

  static const char *fixTypeString[] = {
      "NONE",
      "TIME",
      "2D",
      "3D",
  };

  static const char *fixQualityString[] = {
      "",
      "",
      "/DIFFERENTIAL",
      "/PRECISE",
      "/RTK_FIXED",
      "/RTK_FLOAT",
      "/ESTIMATED",
      "/MANUAL",
      "/SIMULATION",
  };


// battery voltage monitor definitions
float VDDA, VBAT, VBUS, STM32L0Temp;


// BME280 definitions
/* Specify BME280 configuration
 *  Choices are:
 P_OSR_01, P_OSR_02, P_OSR_04, P_OSR_08, P_OSR_16 // pressure oversampling
 H_OSR_01, H_OSR_02, H_OSR_04, H_OSR_08, H_OSR_16 // humidity oversampling
 T_OSR_01, T_OSR_02, T_OSR_04, T_OSR_08, T_OSR_16 // temperature oversampling
 full, BW0_223ODR,BW0_092ODR, BW0_042ODR, BW0_021ODR // bandwidth at 0.021 x sample rate
 BME280Sleep, forced, forced2, normal //operation modes
 t_00_5ms = 0, t_62_5ms, t_125ms, t_250ms, t_500ms, t_1000ms, t_10ms, t_20ms // determines sample rate
 */
uint8_t Posr = P_OSR_01, Hosr = H_OSR_01, Tosr = T_OSR_01, Mode = Sleep, IIRFilter = full, SBy = t_1000ms;     // set pressure amd temperature output data rate

int32_t rawPress, rawTemp, rawHumidity, compTemp;   // pressure, humidity, and temperature raw count output for BME280
uint32_t compHumidity, compPress;                   // variables to hold compensated BME280 humidity and pressure values
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

BME280 BME280; // instantiate BME280 class


//BMA280 definitions
#define BMA280_intPin1 3   // interrupt1 pin definitions
#define BMA280_intPin2 2   // interrupt2 pin definitions

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      BW_7_81Hz, BW_15_63Hz, BW_31_25Hz, BW_62_5Hz, BW_125Hz, BW_250Hz, BW_500Hz, BW_1000Hz
      normal_Mode, deepSuspend_Mode, lowPower_Mode, suspend_Mode
      sleep_0_5ms, sleep_1ms, sleep_2ms, sleep_4ms, sleep_6ms, sleep_10ms, sleep_25ms, sleep_50ms, sleep_100ms, sleep_500ms, sleep_1000ms
*/ 
uint8_t Ascale = AFS_2G, BW = BW_250Hz, power_Mode = lowPower_Mode, sleep_dur = sleep_1000ms;
uint8_t low_power_Mode = lp_mode_1, motion_threshold = 20, tapStatus, tapType;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 

// Logic flags to keep track of device states
bool BMA280_wake_flag = true;
bool BMA280_sleep_flag = false;
bool InMotion = false;
bool ActivityOn = true;

BMA280 BMA280(BMA280_intPin1, BMA280_intPin2); // instantiate BMA280 class


// SPI Flash: 64 MBit (8 MByte) SPI Flash 32,768, 256-byte pages
#define csPin 25 // SPI Flash chip select pin

uint16_t page_number = 0;     // set the page number for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write

SPIFlash SPIFlash(csPin);


void setup()
{
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX); 

  LoRaWAN.getDevEui(buffer, 18);
  Serial.print("STM32L0 Device EUI = "); Serial.println(buffer); 
  
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with blue led off, since active LOW

  pinMode(BMA280_intPin1, INPUT);  // define BMA280 wake and sleep interrupt pins as L082 inputs
  pinMode(BMA280_intPin2, INPUT);
  
  Wire.begin(); // set master mode on default pins 14/15
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);

  BME280.I2Cscan(); // should detect BME280 at 0x77 and BMA280 at 0x18 
  delay(1000);

  GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ); // Start GNSS
  while (GNSS.busy()) { } // wait for begin to complete

  GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GLONASS); // choose satellites
  while (GNSS.busy()) { } // wait for begin to complete

  GNSS.setAntenna(GNSS.ANTENNA_INTERNAL); // GNSS.ANTENNA_INTERNAL or GNSS.ANTENNA_EXTERNAL

  GNSS.enableWakeup();

  pinMode(csPin, OUTPUT); // set SPI chip select as L082 output
  digitalWrite(csPin, HIGH);
  
  // check SPI Flash ID
  SPIFlash.init();      // start SPI
  SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state
  SPIFlash.getChipID(); // Verify SPI flash communication
  SPIFlash.powerDown(); // power down SPI flash

  // Set the RTC time
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);

  // Set the RTC date
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);

  VDDA = STM32L0.getVDDA();
  VBUS = STM32L0.getVBUS();
  VBAT = STM32L0.getVBAT();
  STM32L0Temp = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
  if(VBUS ==  1)  Serial.println("USB Connected!"); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
  Serial.println(" ");
  

  // Read the BMS280 Chip ID register, this is a good test of communication
  Serial.println("BMA280 accelerometer...");
  byte c = BMA280.getChipID();  // Read CHIP_ID register for BMA280
  Serial.print("BMA280 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0xFB, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte d = BME280.getChipID();  // Read WHO_AM_I register for BME280
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x60, HEX);
  Serial.println(" ");
  delay(1000);  
  
  if(c == 0xFB && d == 0x60) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   Serial.println("BMA280 + BME280 are online..."); Serial.println(" ");
   
  aRes = BMA280.getAres(Ascale);                                     // get sensor resolutions, only need to do this once
  BMA280.resetBMA280();                                              // software reset before initialization
  delay(100);      
  BMA280.selfTestBMA280();                                           // perform sensor self test
  BMA280.resetBMA280();                                              // software reset before initialization
  delay(1000);                                                       // give some time to read the screen
  BMA280.initBMA280(Ascale, BW, normal_Mode, sleep_dur);             // initialize sensor in normal mode for calibration
  BMA280.fastCompensationBMA280();                                   // quickly estimate offset bias
  BMA280.initBMA280_MotionManager(Ascale, BW, power_Mode, sleep_dur, low_power_Mode, motion_threshold); // Initialize sensor in desired mode for application                     

  BME280.resetBME280();                                              // reset BME280 before initilization
  delay(100);

  BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy);         // Initialize BME280 altimeter
  BME280.BME280forced();                                             // get initial data sample, then go back to sleep
  }
  else 
  {
  if(c != 0xFB) Serial.println(" BMA280 not functioning!");
  if(d != 0x60) Serial.println(" BME280 not functioning!");     
  }

   // set alarm to update the RTC periodically
  RTC.setAlarmTime(12, 0, 0);
  RTC.enableAlarm(RTC.MATCH_SS);  // alarm once per minute
//    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

  RTC.attachInterrupt(alarmMatch);

  attachInterrupt(BMA280_intPin1, myinthandler1, RISING);  // define interrupt for INT1 pin output of BMA280
  attachInterrupt(BMA280_intPin2, myinthandler2, RISING);  // define interrupt for INT2 pin output of BMA280


  // Configuree LoRaWAN connection
  /*
    - Asia       AS923
    - Australia  AU915
    - Europe     EU868
    - India      IN865
    - Korea      KR920
    - US         US915 (64 + 8 channels)
   */
    LoRaWAN.begin(US915);
    LoRaWAN.setADR(false);
    LoRaWAN.setDataRate(1);
    LoRaWAN.setTxPower(10);
    LoRaWAN.setSubBand(1); // 1 for MTCAP, 2 for TT gateways

    LoRaWAN.joinOTAA(appEui, appKey, devEui);

    LoRaTimer.start(callbackLoRaTx, 300000, 600000);      //  10 minute period, delayed 5 minutes

    NoMotionActivityTimer.start(callbackNoMotionActivity, 100000, 7200000);    // low  freq (two hours) timer
    InMotionActivityTimer.start(callbackInMotionActivity, 100000, 7200000);    // high freq (one minute) timer

// for testing
//    LoRaTimer.start(callbackLoRaTx, 60000, 60000);      //  1 minute period, delayed 1 minute

//    NoMotionActivityTimer.start(callbackNoMotionActivity, 0, 600000);    // low  freq (two hours) timer
//    InMotionActivityTimer.start(callbackInMotionActivity, 100000,   60000);    // high freq (one minute) timer
    
    /* end of setup */

}
/* 
 *  
 * Everything in the main loop is based on interrupts, so that 
 * if there has not been an interrupt event the STM32L082 should be in STOP mode
*/
 
void loop()
{
    // use BMA280 wake and sleep motion detection to detect motion state
    if(BMA280_wake_flag == true) 
    {
      BMA280_wake_flag = false;      // clear the flag
      InMotion = true;               // set motion state latch
      BMA280.activateNoMotionInterrupt();                                                                // Re-enable BMA280 no motion interrupt
      attachInterrupt(BMA280_intPin2, myinthandler2, RISING);   
    }

    if(BMA280_sleep_flag == true)
    {
      BMA280_sleep_flag = false;    // clear the flag
      detachInterrupt(BMA280_intPin2);                                                                  // Detach the BMA280 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
      BMA280.deactivateNoMotionInterrupt();   
    } /* end of sleep/wake detect */


  /*GNSS*/
  if (GNSS.location(myLocation))
  {
  Serial.print("LOCATION: ");
  Serial.print(fixTypeString[myLocation.fixType()]);

  if (myLocation.fixType() != GNSSLocation::TYPE_NONE)
  {
      Hour   = myLocation.hours();
      Minute = myLocation.minutes();
      Second = myLocation.seconds();
      Year   = myLocation.year();
      Month  = myLocation.month();
      Day    = myLocation.day();
      
      Serial.print(fixQualityString[myLocation.fixQuality()]);
      Serial.print(" ");
      Serial.print(myLocation.year());
      Serial.print("/");
      Serial.print(myLocation.month());
      Serial.print("/");
      Serial.print(myLocation.day());
      Serial.print(" ");
      if (myLocation.hours() <= 9) {Serial.print("0");}
      Serial.print(myLocation.hours());
      Serial.print(":");
      if (myLocation.minutes() <= 9) {Serial.print("0");}
      Serial.print(myLocation.minutes());
      Serial.print(":");
      if (myLocation.seconds() <= 9) {Serial.print("0");}
      Serial.print(myLocation.seconds());
      Serial.print(".");
      if (myLocation.millis() <= 9) {Serial.print("0");}
      if (myLocation.millis() <= 99) {Serial.print("0");}
      Serial.print(myLocation.millis());

      if (myLocation.leapSeconds() != GNSSLocation::LEAP_SECONDS_UNDEFINED) {
                Serial.print(" ");
                Serial.print(myLocation.leapSeconds());
                if (!myLocation.fullyResolved()) {
                    Serial.print("D");
                }
       }

      if (myLocation.fixType() != GNSSLocation::TYPE_TIME)
      {
      Lat = myLocation.latitude();
      myLocation.latitude(latOut);
      Long = myLocation.longitude();
      myLocation.longitude(longOut);
      Alt = myLocation.altitude();
      EPE = myLocation.ehpe(); // use this as accuracy figure of merit
      Serial.print(" LLA=");
      Serial.print(Lat, 7);
      Serial.print(",");
      Serial.print(Long, 7);
      Serial.print(",");
      Serial.print(Alt, 3);
      Serial.print(" EPE=");
      Serial.print(EPE, 3);
      Serial.print(",");
      Serial.print(myLocation.evpe(), 3);
      Serial.print(" SATELLITES=");
      Serial.print(myLocation.satellites());
      Serial.print(" DOP=");
      Serial.print(myLocation.hdop(), 2);
      Serial.print(",");
      Serial.print(myLocation.vdop(), 2);
      Serial.println();
    
        // Put the CAM M8Q to sleep once 3D fix with sufficient accuracy is obtained
        if( (myLocation.fixType() != GNSSLocation::TYPE_2D) && (EPE <= 30.0f) && myLocation.fullyResolved())  // 10 is about as low as one should go, 50 is acceptable
        {
            if (!isTracking)
                {
                    isTracking = true;
                    
                    Serial.println("***GNSS go to sleep!***");
                    GNSS.suspend(); // once we have a good 3D location fix put CAM M8Q to sleep
                    callbackLoRaTx();  // update dashboard/backend via LoRaWAN
                }
           
         }

      }

  } 

  Serial.println();

  } /* end of GNSS Location handling */

    if (GNSS.satellites(mySatellites))
    {

    Serial.print("SATELLITES: ");
    Serial.print(mySatellites.count());
  
    Serial.println();

    for (unsigned int index = 0; index < mySatellites.count(); index++)
    {
  unsigned int svid = mySatellites.svid(index);

  if ((svid >= 1) && (svid <= 32))
  {
      Serial.print("    ");

      if (svid <= 9)
      {
    Serial.print("  G");
    Serial.print(svid);
      }
      else
      {
    Serial.print(" G");
    Serial.print(svid);
      }
  }
  else if ((svid >= 65) && (svid <= 96))
  {
      Serial.print("    ");

      if ((svid - 64) <= 9)
      {
    Serial.print("  R");
    Serial.print(svid - 64);
      }
      else
      {
    Serial.print(" R");
    Serial.print(svid - 64);
      }
  }
  else if ((svid >= 120) && (svid <= 158))
  {
      Serial.print("    ");
      Serial.print("S");
      Serial.print(svid);
  }
  else if ((svid >= 173) && (svid <= 182))
  {
      Serial.print("    ");
      Serial.print("  I");
      Serial.print(svid - 172);
  }
  else if ((svid >= 193) && (svid <= 197))
  {
      Serial.print("    ");
      Serial.print("  Q");
      Serial.print(svid - 192);
  }
  else if ((svid >= 211) && (svid <= 246))
  {
      Serial.print("    ");

      if ((svid - 210) <= 9)
      {
    Serial.print("  E");
    Serial.print(svid - 210);
      }
      else
      {
    Serial.print(" E");
    Serial.print(svid - 210);
      }
  }
  else if (svid == 255)
  {
      Serial.print("    ");
      Serial.print("R???");
  }
  else
  {
      continue;
  }

  Serial.print(": SNR=");
  Serial.print(mySatellites.snr(index));
  Serial.print(", ELEVATION=");
  Serial.print(mySatellites.elevation(index));
  Serial.print(", AZIMUTH=");
  Serial.print(mySatellites.azimuth(index));

  if (mySatellites.unhealthy(index)) {
      Serial.print(", UNHEALTHY");
  }

  if (mySatellites.almanac(index)) {
      Serial.print(", ALMANAC");
  }

  if (mySatellites.ephemeris(index)) {
      Serial.print(", EPHEMERIS");
  }

  if (mySatellites.autonomous(index)) {
      Serial.print(", AUTONOMOUS");
  }

  if (mySatellites.correction(index)) {
      Serial.print(", CORRECTION");
  }

  if (mySatellites.acquired(index)) {
      Serial.print(", ACQUIRED");
  }

  if (mySatellites.locked(index)) {
      Serial.print(", LOCKED");
  }

  if (mySatellites.navigating(index)) {
      Serial.print(", NAVIGATING");
  }

  Serial.println();
    }

} /* end of GNSS Satellites handling */


  /*RTC*/
  if (alarmFlag) { // update serial output and log to SPI flash whenever there is an RTC alarm
    alarmFlag = false;
 
//    if(SerialDebug) {
//    Serial.print("ax = ");  Serial.print((int)1000*ax);  
//    Serial.print(" ay = "); Serial.print((int)1000*ay); 
//    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
//    }

    VBAT = STM32L0.getVBAT();
    if(SerialDebug) {
      Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
    }

//    tempCount = BMA280.readBMA280TempData();  // Read the accel chip temperature adc values
//    temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
//    Serial.print("Accel temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        

    // BME280 Data
    BME280.BME280forced();  // get one data sample, then go back to sleep
    
    rawTemp =  BME280.readBME280Temperature();
    compTemp = BME280.BME280_compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    rawPress =  BME280.readBME280Pressure();
    compPress = BME280.BME280_compensate_P(rawPress);
    pressure = (float) compPress/25600.0f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    rawHumidity =  BME280.readBME280Humidity();
    compHumidity = BME280.BME280_compensate_H(rawHumidity);
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH
 
    if(SerialDebug){
    Serial.println("BME280:");
    Serial.print("Altimeter temperature = "); 
    Serial.print( temperature_C, 2); 
    Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Altimeter temperature = "); 
    Serial.print(temperature_F, 2); 
    Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter pressure = "); 
    Serial.print(pressure, 2);  
    Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); 
    Serial.print(altitude, 2); 
    Serial.println(" feet");
    Serial.print("Altimeter humidity = "); 
    Serial.print(humidity, 1);  
    Serial.println(" %RH");// pressure in millibar
    Serial.println(" ");
    }

    // Read RTC
    Serial.println("RTC:");
    RTC.getDate(day, month, year);
    RTC.getTime(hours, minutes, seconds, subSeconds);

    milliseconds = ((subSeconds >> 17) * 1000 + 16384) / 32768;

    Serial.print("RTC Time = ");
    if (hours < 10)   {Serial.print("0");Serial.print(hours); } else Serial.print(hours);
    Serial.print(":");
    if (minutes < 10) {Serial.print("0"); Serial.print(minutes); } else Serial.print(minutes);
    Serial.print(":");
    if (seconds < 10) {Serial.print("0"); Serial.print(seconds); } else Serial.print(seconds);
    Serial.print(".");
        if (milliseconds <= 9) {
            Serial.print("0");
        }
        if (milliseconds <= 99) {
            Serial.print("0");
        }
    Serial.print(milliseconds);
    Serial.println(" ");

    Serial.print("RTC Date = ");
    Serial.print(year); Serial.print(":"); Serial.print(month); Serial.print(":"); Serial.println(day);
    Serial.println();
      
    // Send some data to the SPI flash
      if (sector_number < 8 && page_number < 0x7FFF) { // 32,768 256-byte pages in a 8 MByte flash
        flashPage[sector_number * 32 + 0]  = (latOut & 0xFF000000) >> 24;  // latitude in bytes
        flashPage[sector_number * 32 + 1]  = (latOut & 0x00FF0000) >> 16;
        flashPage[sector_number * 32 + 2]  = (latOut & 0x0000FF00) >> 8;
        flashPage[sector_number * 32 + 3]  = (latOut & 0x000000FF);
        flashPage[sector_number * 32 + 4]  = (longOut & 0xFF000000) >> 24; // longitude in bytes
        flashPage[sector_number * 32 + 5]  = (longOut & 0x00FF0000) >> 16;
        flashPage[sector_number * 32 + 6]  = (longOut & 0x0000FF00) >> 8;
        flashPage[sector_number * 32 + 7]  = (longOut & 0x000000FF);
        flashPage[sector_number * 32 + 8] =  (compTemp & 0xFF000000) >> 24;
        flashPage[sector_number * 32 + 9] =  (compTemp & 0x00FF0000) >> 16;
        flashPage[sector_number * 32 + 10] = (compTemp & 0x0000FF00) >> 8;
        flashPage[sector_number * 32 + 11] = (compTemp & 0x000000FF);
        flashPage[sector_number * 32 + 12] = (compHumidity & 0xFF000000) >> 24;
        flashPage[sector_number * 32 + 13] = (compHumidity & 0x00FF0000) >> 16;
        flashPage[sector_number * 32 + 14] = (compHumidity & 0x0000FF00) >> 8;
        flashPage[sector_number * 32 + 15] = (compHumidity & 0x000000FF);
        flashPage[sector_number * 32 + 16] = (compPress & 0xFF000000) >> 24;
        flashPage[sector_number * 32 + 17] = (compPress & 0x00FF0000) >> 16;
        flashPage[sector_number * 32 + 18] = (compPress & 0x0000FF00) >> 8;
        flashPage[sector_number * 32 + 19] = (compPress & 0x000000FF);
        flashPage[sector_number * 32 + 20] = seconds;
        flashPage[sector_number * 32 + 21] = minutes;
        flashPage[sector_number * 32 + 22] = hours;
        flashPage[sector_number * 32 + 23] = day;
        flashPage[sector_number * 32 + 24] = month;
        flashPage[sector_number * 32 + 25] = year;
        flashPage[sector_number * 32 + 26] =  ( (uint16_t (Alt * 10.0f)) & 0xFF00) >> 8;  // MSB GPS altitude
        flashPage[sector_number * 32 + 27] =  ( (uint16_t (Alt * 10.0f)) & 0x00FF);       // LSB GPS altitude
        flashPage[sector_number * 32 + 28] =  ( (uint16_t (VBAT * 100.0f)) & 0xFF00) >> 8;
        flashPage[sector_number * 32 + 29] =  ( (uint16_t (VBAT * 100.0f)) & 0x00FF);

        sector_number++;
      }
      else if (sector_number == 8 && page_number < 0x7FFF)
      {
          SPI.begin();           // When exiting STOP mode, re-enable the SPI peripheral
          SPIFlash.powerUp();
          SPIFlash.flash_page_program(flashPage, page_number);
          Serial.print("***Wrote flash page: "); Serial.println(page_number);
          digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH); // indicate when flash page is written
          sector_number = 0;
          page_number++;
          SPIFlash.powerDown();  // Put SPI flash into power down mode
          SPI.end();             // End SPI peripheral to save power in STOP mod     
      }
      else
      {
        Serial.println("Reached last page of SPI flash!"); Serial.println("Data logging stopped!");
      }

     digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH);
        
    } // end of alarm section
    
 
    STM32L0.stop();        // Enter STOP mode and wait for an interrupt
   
}  /* end of loop*/


/* Useful functions */

void callbackLoRaTx(void)
{     

    if (!LoRaWAN.busy() && LoRaWAN.joined())
     {
        myLPP.reset();
        myLPP.addTemperature(1, temperature_C);
        myLPP.addRelativeHumidity(2, humidity);
        myLPP.addBarometricPressure(3, pressure);
        myLPP.addAnalogInput(4, VBAT);
        myLPP.addGPS(5, Lat, Long, Alt);

        LoRaWAN.sendPacket(myLPP.getBuffer(), myLPP.getSize());
     }
    
}


void callbackNoMotionActivity(void)
{
    GNSS.resume();
    isTracking = false;
}


void callbackInMotionActivity(void)
{
  if(InMotion)
  {
   InMotion = false;
   GNSS.resume();
   isTracking = false;
  }
}


void myinthandler1()
{
  BMA280_wake_flag = true; 
  STM32L0.wakeup();
  Serial.println("BMA280 is awake!");
}


void myinthandler2()
{
  BMA280_sleep_flag = true;
  STM32L0.wakeup();
  Serial.println("BMA280 is asleep!");
}


void alarmMatch()
{
  alarmFlag = true;
  STM32L0.wakeup();
}
