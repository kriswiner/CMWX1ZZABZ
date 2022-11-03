/* Gnat v.03a Asset Tracker contains:
   CMWX1ZZABZ-078 (STM32L082 and SX1276)
   LIS2DW12 accelerometer sensor
   MAX M8Q-10 Concurrent GNSS engine

   https://hackaday.io/project/25790-asset-tracker

   Idea is ultra-low power for longest LiPo battery life so I would run this with
   4.2 MHz clock speed; this reduction plus use of STM32 stop mode means no serial
   through the USB.  

   Copyright 2022 Tlera Corporation

   For unlimited distribution with attribution

   This example code is in the public domain.
*/
#include <STM32L0.h>
#include "LoRaWAN.h"
#include "TimerMillis.h"
#include "GNSS.h"
#include "RTC.h"
#include "LIS2DW12.h"
#include "CayenneLPP.h"

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

// Gnat Asset Tracker gnat1
const char *appEui = "70B3D57ED000964D";
const char *appKey = "6BF7C77E61444EA4608D4353207F25BE";
const char *devEui = "3739323254377a09";

CayenneLPP myLPP(64);

// Gnat pin assignments
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
volatile bool ppsFlag = false, firstSync = false, alarmFlag = true;
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

//LIS2DW12 definitions
#define LIS2DW12_intPin1   A4    // interrupt1 pin definitions, wake-up from STANDBY pin
#define LIS2DW12_intPin2    3    // interrupt2 pin definitions, data ready or sleep interrupt

// Specify sensor parameters //
LPMODE   lpMode = LIS2DW12_LP_MODE_1;      // choices are low power modes 1, 2, 3, or 4
MODE     mode   = LIS2DW12_MODE_LOW_POWER; // choices are low power, high performance, and one shot modes
ODR      odr    = LIS2DW12_ODR_12_5_1_6HZ; //  1.6 Hz in lpMode, max is 200 Hz in LpMode
FS       fs     = LIS2DW12_FS_2G;          // choices are 2, 4, 8, or 16 g
BW_FILT  bw     = LIS2DW12_BW_FILT_ODR2;   // choices are ODR divided by 2, 4, 10, or 20
FIFOMODE fifoMode = BYPASS;                // capture 32 samples of data before wakeup event, about 2 secs at 25 Hz
bool lowNoise = false;                     // low noise or lowest power

float aRes = 0;         // Sensor data scale in mg/LSB
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // holds accel bias offsets
float stress[3];        // holds results of the self test
uint8_t status = 0, wakeSource = 0, FIFOstatus = 0, numFIFOSamples = 0;

// Logic flags to keep track of device states
volatile bool LIS2DW12_wake_flag = false;
volatile bool LIS2DW12_sleep_flag = false;
volatile bool InMotion = false;

LIS2DW12 LIS2DW12(&i2c_0); // instantiate LIS2DW12 class


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

  pinMode(LIS2DW12_intPin1, INPUT);  // define LIS2DW12 wake and sleep interrupt pins as L082 inputs
  pinMode(LIS2DW12_intPin2, INPUT);

//  pinMode(pps, INPUT); // select pps as input from MAX M8Q

  pinMode(GNSS_backup, OUTPUT);
  digitalWrite(GNSS_backup, HIGH);

  /* initialize two wire bus */
  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L0
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(1000);

  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                // should detect LIS2DW12 at 0x19
  delay(1000);
  
  /* Initialize and configure GNSS */
  GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ); // Start GNSS
  while (GNSS.busy()) { } // wait for begin to complete

  GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GLONASS); // choose satellites
  while (GNSS.busy()) { } // wait for set to complete

  GNSS.setAntenna(GNSS.ANTENNA_EXTERNAL);  
  while (GNSS.busy()) { } // wait for set to complete

  GNSS.enableWakeup();
  while (GNSS.busy()) { } // wait for set to complete

  // Read the LIS2DW12 Chip ID register, this is a good test of communication
  Serial.println("LIS2DW12 accelerometer...");
  byte LIS2DW12_ID = LIS2DW12.getChipID();  // Read CHIP_ID register for LIS2DW12
  Serial.print("LIS2DW12 "); Serial.print("I AM "); Serial.print(LIS2DW12_ID, HEX); Serial.print(" I should be "); Serial.println(0x44, HEX);
  Serial.println(" ");
  delay(1000); 

  if(LIS2DW12_ID == 0x44) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   Serial.println("LIS2DW12 is online..."); Serial.println(" ");
   
   LIS2DW12.reset();                                                // software reset before initialization
   delay(100);      

   LIS2DW12.selfTest(stress);                                       // perform sensor self test
   Serial.print("x-axis self test = "); Serial.print(stress[0], 1); Serial.println("mg, should be between 70 and 1500 mg");
   Serial.print("y-axis self test = "); Serial.print(stress[1], 1); Serial.println("mg, should be between 70 and 1500 mg");
   Serial.print("z-axis self test = "); Serial.print(stress[2], 1); Serial.println("mg, should be between 70 and 1500 mg");
   delay(1000);                                                     // give some time to read the screen

   LIS2DW12.reset();                                                // software reset before initialization
   delay(100);                                                     

   aRes = 0.000244f * (1 << fs);                                    // scale resolutions per LSB for the sensor at 14-bit data 

   Serial.println("hold flat and motionless for bias calibration");
   delay(5000);
   LIS2DW12.Compensation(fs, odr, mode, lpMode, bw, lowNoise, offset); // quickly estimate offset bias in normal mode
   Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
   Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
   Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg");

   LIS2DW12.init(fs, odr, mode, lpMode, bw, lowNoise);               // Initialize sensor in desired mode for application                     
   LIS2DW12.configureFIFO(fifoMode, 0x1F); // 32 levels of data
   delay(1000); // let sensor settle
   }
  else 
  {
   if(LIS2DW12_ID != 0x44) Serial.println(" LIS2DW12 not functioning!");
  }

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
  VBAT = 1.27f * VDDA * ((float) analogRead(myBat)) / 4096.0f;
  STM32L0Temp = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
  if(VBUS == 1)  Serial.println("USB Connected!"); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
  Serial.println(" ");

  // set alarm to update the RTC periodically
  RTC.setAlarmTime(12, 0, 0);
  RTC.enableAlarm(RTC.MATCH_SS);  // alarm once per minute

  RTC.attachInterrupt(alarmMatch);

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
    LoRaWAN.setSubBand(2); // 1 for MTCAP, 2 for TTN servers

    LoRaWAN.joinOTAA(appEui, appKey, devEui);

//    LoRaTimer.start(callbackLoRaTx, 300000, 600000);      //  10 minute period, delayed 5 minutes

//    NoMotionActivityTimer.start(callbackNoMotionActivity, 100000, 7200000);    // low  freq (two hours) timer
//    InMotionActivityTimer.start(callbackInMotionActivity, 100000,  600000);    // high freq (ten minute) timer

// For testing
    LoRaTimer.start(callbackLoRaTx, 60000, 600000);      //  10 minute period, delayed 1 minutes

    NoMotionActivityTimer.start(callbackNoMotionActivity, 0, 300000);        // low  freq (five minute) timer
//    InMotionActivityTimer.start(callbackInMotionActivity, 100000,   60000);  // high freq (one minute) timer

//   attachInterrupt(pps, CAMM8QintHandler, RISING);
   attachInterrupt(LIS2DW12_intPin1, myinthandler1, RISING);  // define wake-up interrupt for INT1 pin output of LIS2DW12
   attachInterrupt(LIS2DW12_intPin2, myinthandler2, RISING);  // define data ready interrupt for INT2 pin output of LIS2DW12
   LIS2DW12.getStatus(); // read status of interrupts to clear

    /* end of setup */
}

void loop()
{

  /* LIS2DW12 sleep/wake detect*/
  if(LIS2DW12_wake_flag)
  {
   LIS2DW12_wake_flag = false; // clear the wake flag
   InMotion = true;          // set motion state latch
   LIS2DW12.activateNoMotionInterrupt();  
   attachInterrupt(LIS2DW12_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of LIS2DW12 
  }

  if(LIS2DW12_sleep_flag)
  {
   LIS2DW12_sleep_flag = false;            // clear the sleep flag
   detachInterrupt(LIS2DW12_intPin2);       // Detach the LIS2DW12 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
   LIS2DW12.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power 
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
      if( (myLocation.fixType() != GNSSLocation::TYPE_2D) && (EPE <= 20.0f) && myLocation.fullyResolved())  // 10 is about as low as one should go, 50 is acceptable
        {
            if (!isTracking)
                {
                    isTracking = true;
                    
                    Serial.println("***GNSS go to sleep!***");
                    GNSS.suspend();    // once we have a good 3D location fix put M8Q to sleep
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
    
    if(SerialDebug && LIS2DW12_wake_flag) {
      
     LIS2DW12.readAccelData(accelCount); // get 14-bit signed accel data

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

    tempCount = LIS2DW12.readTempData();  // Read the accel chip temperature adc values
    temperature =  ((float) tempCount) + 25.0f; // 8-bit accel chip temperature in degrees Centigrade
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
    Serial.print("GNSS resume call next!");
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
  LIS2DW12_wake_flag = true; 
  STM32L0.wakeup();
  Serial.println("** LIS2DW12 is awake! **");
}


void myinthandler2()
{
  LIS2DW12_sleep_flag = true;
  STM32L0.wakeup();
  Serial.println("** LIS2DW12 is asleep! **");
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
