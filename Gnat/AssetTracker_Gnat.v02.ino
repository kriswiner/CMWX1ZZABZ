/* Gnat Asset Tracker contains:
   CMWX1ZZABZ (STM32L082 and SX1276)
   BMA400 accelerometer sensor
   MAX M8Q Concurrent GNSS engine

   https://hackaday.io/project/25790-asset-tracker

   Idea is ultra-low power for longest LiPo battery life so I would run this with
   4.2 MHz clock speed; this reduction plus use of STM32 stop mode means no serial
   through the USB.  

   Copyright 2018 Tlera Corporation

   For unlimited distribution with attribution

   This example code is in the public domain.
*/
#include <STM32L0.h>
#include "LoRaWAN.h"
#include "TimerMillis.h"
#include "GNSS.h"
#include <RTC.h>
#include "BMA400.h"
#include "CayenneLPP.h"

// Gnat Asset Tracker gnat1
const char *appEui = "70B3D57ED000964D";
const char *appKey = "7DE66B18F7105B19A1427AFEB2514597";
const char *devEui = "3739323254377a09";

CayenneLPP myLPP(64);

// Cricket pin assignments
#define myLed    10 // blue led 
#define myBat    A1 // LiPo battery ADC
#define myBat_en  2 // LiPo battery monitor enable

uint8_t LoRaData[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

TimerMillis LoRaTimer;

GNSSLocation myLocation;
GNSSSatellites mySatellites;

volatile bool isTracking = false;

TimerMillis NoMotionActivityTimer;  // instantiate low-frequency timer
TimerMillis InMotionActivityTimer;  // instantiate high-frequency timer

uint32_t UID[3] = {0, 0, 0}; 
char buffer[32];

bool SerialDebug = true;

// MAX M8Q GNSS configuration
#define GNSS_en      5     // enable for GNSS 3.0 V LDO
#define pps          4     // 1 Hz fix pulse
#define GNSS_backup A0     // RTC backup for MAX M8Q

uint16_t Hour = 1, Minute = 1, Second = 1, Millisec, Year = 1, Month = 1, Day = 1;
uint8_t hours = 12, minutes = 0, seconds = 0, year = 1, month = 1, day = 1;
uint32_t subSeconds, milliseconds;
bool ppsFlag = false, firstSync = false, alarmFlag = true;
uint16_t count = 0, fixType = 0, fixQuality;
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

//BMA400 definitions
#define BMA400_intPin1 A4   // interrupt1 pin definitions, wake-up from STANDBY pin
#define BMA400_intPin2  3   // interrupt2 pin definitions, data ready or sleep interrupt

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      SR_15_5Hz, SRW_25Hz, SR_50Hz, SR_100Hz, SR_200Hz, SR_400Hz, SR_800Hz 
      sleep_Mode, lowpower_Mode, normal_Mode 
      osr0 (lowest power, lowest oversampling,lowest accuracy), osr1, osr2, osr3 (highest power, highest oversampling, highest accuracy)
      acc_filt1 (variable filter), acc_filt2 (fixed 100 Hz filter), acc_filt_lp (fixed 100 Hz filter, 1 Hz bandwidth)
*/ 
uint8_t Ascale = AFS_2G, SR = SR_200Hz, power_Mode = lowpower_Mode, OSR = osr0, acc_filter = acc_filt2;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // accel bias offsets

// Logic flags to keep track of device states
bool BMA400_wake_flag = false;
bool BMA400_sleep_flag = false;
bool BMA400_newData_flag = false;
bool InMotion = false;
bool ActivityOn = true;

BMA400 BMA400(BMA400_intPin1, BMA400_intPin2); // instantiate BMA400 class


void setup()
{
  /* Enable USB UART */
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  /* Get LoRa/LoRaWAN ID for SX1276 */
  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX); 

  LoRaWAN.getDevEui(buffer, 18);
  Serial.print("STM32L0 Device EUI = "); Serial.println(buffer); 
  
  /* configure IO pins */
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with blue led off, since active LOW

  pinMode(myBat_en, OUTPUT);
  pinMode(myBat, INPUT);    // set up ADC battery voltage monitor pin
  analogReadResolution(12); // use 12-bit ADC resolution

  pinMode(BMA400_intPin1, INPUT);  // define BMA400 wake and sleep interrupt pins as L082 inputs
  pinMode(BMA400_intPin2, INPUT);

  pinMode(pps, INPUT); // select pps as input from MAX M8Q

  /* initialize wire bus */
  Wire.begin(); // set master mode on default pins 14/15
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);

  BMA400.I2Cscan(); // should detect BMA400 at 0x18 
  delay(1000);
  
  /* Initialize and configure GNSS */
  GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ); // Start GNSS
  while (GNSS.busy()) { } // wait for begin to complete

  GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GLONASS); // choose satellites
  while (GNSS.busy()) { } // wait for begin to complete

  GNSS.setAntenna(GNSS.ANTENNA_EXTERNAL);  

  GNSS.enableWakeup();

  /* Set the RTC time */
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);

  // Set the RTC date
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  
  /* Check internal STML082 and battery power configuration */
  VDDA = STM32L0.getVDDA();
  VBUS = STM32L0.getVBUS();
  VBAT = 1.27f * 3.30f * ((float) analogRead(myBat)) / 4096.0f;
  STM32L0Temp = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
  if(VBUS ==  1)  Serial.println("USB Connected!"); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
  Serial.println(" ");
  

  // Read the BMA400 Chip ID register, this is a good test of communication
  Serial.println("BMA400 accelerometer...");
  byte c = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial.print("BMA400 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x90, HEX);
  Serial.println(" ");
  delay(1000); 

  if(c == 0x90) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   Serial.println("BMA400 is online..."); Serial.println(" ");
   
   aRes = BMA400.getAres(Ascale);                                       // get sensor resolutions, only need to do this once
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(100);      
   BMA400.selfTestBMA400();                                             // perform sensor self test
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(1000);                                                         // give some time to read the screen
   BMA400.CompensationBMA400(Ascale, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
   BMA400.initBMA400(Ascale, SR, power_Mode, OSR, acc_filter);          // Initialize sensor in desired mode for application                     

  }
  else 
  {
  if(c != 0x90) Serial.println(" BMA400 not functioning!");
  }

   // set alarm to update the RTC periodically
  RTC.setAlarmTime(0, 0, 0);
//  RTC.enableAlarm(RTC.MATCH_SS);  // alarm once per minute
    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

  RTC.attachInterrupt(alarmMatch);

  attachInterrupt(BMA400_intPin1, myinthandler1, RISING);  // define wake-up interrupt for INT1 pin output of BMA400
  attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // define data ready interrupt for INT2 pin output of BMA400
  BMA400.getStatus(); // read status of interrupts to clear

  attachInterrupt(pps, CAMM8QintHandler, RISING);


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
    InMotionActivityTimer.start(callbackInMotionActivity, 100000,   60000);    // high freq (one minute) timer

// For testing
//    LoRaTimer.start(callbackLoRaTx, 60000, 60000);      //  1 minute period, delayed 1 minutes

//    NoMotionActivityTimer.start(callbackNoMotionActivity, 0, 360000);        // low  freq (five minute) timer
//    InMotionActivityTimer.start(callbackInMotionActivity, 100000,   60000);  // high freq (one minute) timer
    
    /* end of setup */
}

void loop()
{

  /* BMA400 sleep/wake detect*/
  if(BMA400_wake_flag)
  {
   BMA400_wake_flag = false; // clear the wake flag
   InMotion = true;          // set motion state latch
   BMA400.activateNoMotionInterrupt();  
   attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 
  }

  if(BMA400_sleep_flag)
  {
   BMA400_sleep_flag = false;            // clear the sleep flag
   detachInterrupt(BMA400_intPin2);       // Detach the BMA400 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
   BMA400.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power 
  }/* end of sleep/wake detect */

  
  /*GNSS*/
//if(ppsFlag)
//{
//  ppsFlag = false;

  if(GNSS.location(myLocation))
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
    
//   } 
 
 }/* end of GNSS Satellites handling */


  /*RTC*/
  if (alarmFlag) { // update RTC output whenever there is a GNSS pulse
    alarmFlag = false;
    
    if(SerialDebug && BMA400_wake_flag) {
      
     BMA400.readBMA400AccelData(accelCount); // get 12-bit signed accel data

    // Now we'll calculate the accleration value into actual g's
     ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes - offset[1];   
     az = (float)accelCount[2]*aRes - offset[2]; 
     
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    }

    VDDA = STM32L0.getVDDA();
    digitalWrite(myBat_en, HIGH);
    VBAT = 1.27f * VDDA * ((float) analogRead(myBat)) / 4096.0f;
    digitalWrite(myBat_en, LOW);
    STM32L0Temp = STM32L0.getTemperature();
    if(SerialDebug) {
      Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
      Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
      Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
      Serial.println(" ");
    }

    tempCount = BMA400.readBMA400TempData();  // Read the accel chip temperature adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial.print("Accel temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        

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

    digitalWrite(myLed, !digitalRead(myLed)); delay(1); digitalWrite(myLed, !digitalRead(myLed));
        
    } // end of alarm section
    
 
    STM32L0.stop();        // Enter STOP mode and wait for an interrupt
   
}  /* end of loop*/


/* Useful functions */

void callbackLoRaTx(void)
{     
/*     // Send some data via LoRaWAN
      LoRaData[0]  = (uint16_t(temperature_C*100.0) & 0xFF00) >> 8;
      LoRaData[1]  =  uint16_t(temperature_C*100.0) & 0x00FF;
      LoRaData[2] =  (uint16_t(pressure*10.0      ) & 0xFF00) >> 8;   
      LoRaData[3] =   uint16_t(pressure*10.0      ) & 0x00FF;         
      LoRaData[4] =  (uint16_t(humidity*100.0     ) & 0xFF00) >> 8;
      LoRaData[5] =   uint16_t(humidity*100.0     ) & 0x00FF;
      LoRaData[6] =  (uint16_t( (Long + 123.0)*10000.0 ) & 0xFF00) >> 8;
      LoRaData[7] =   uint16_t( (Long + 123.0)*10000.0 ) & 0x00FF;
      LoRaData[8] =  (uint16_t( (Lat   - 37.0)*10000.0 ) & 0xFF00) >> 8;
      LoRaData[9] =   uint16_t( (Lat   - 37.0)*10000.0 ) & 0x00FF;
      LoRaData[10] =  uint8_t(VBAT*50.0); // maximum should be 4.2 * 50 = 210
*/
    if (!LoRaWAN.busy() && LoRaWAN.joined())
     {
//        LoRaWAN.beginPacket(3);
//        LoRaWAN.write(LoRaData, sizeof(LoRaData));
//        LoRaWAN.endPacket();

        myLPP.reset();
        myLPP.addAnalogInput(1, VBAT);
        myLPP.addGPS(2, Lat, Long, Alt);
        myLPP.addTemperature(3, STM32L0Temp);
        myLPP.addTemperature(4, temperature);

        LoRaWAN.sendPacket(myLPP.getBuffer(), myLPP.getSize());
     }
    
}


void callbackNoMotionActivity(void)
{
    GNSS.resume(); // long duty cycle simply resume GNSS after time out
    isTracking = false;
}


void callbackInMotionActivity(void)
{
  if(InMotion) // short duty cycle resume GNSS only if motion has been detected since last GNSS.suspend
  {
   InMotion = false;
   GNSS.resume();
   isTracking = false;
  }
}



void myinthandler1()
{
  BMA400_wake_flag = true; 
  STM32L0.wakeup();
  Serial.println("** BMA400 is awake! **");
}


void myinthandler2()
{
  BMA400_sleep_flag = true;
  STM32L0.wakeup();
  Serial.println("** BMA400 is asleep! **");
}


void CAMM8QintHandler()
{
  ppsFlag = true;
  STM32L0.wakeup();
}


void alarmMatch()
{
  alarmFlag = true;
  STM32L0.wakeup();
}


void syncRTC()
{
  // Set the time
  RTC.setSeconds(Second);
  RTC.setMinutes(Minute);
  if (Hour < 7) {
    RTC.setHours(Hour + 17);
  } else RTC.setHours(Hour - 7);
  RTC.setMinutes(Minute);

  // Set the date
  if (Hour < 7) {
    RTC.setDay(Day - 1);
  } else RTC.setDay(Day);
  RTC.setMonth(Month);
  RTC.setYear(Year - 2000);
}
