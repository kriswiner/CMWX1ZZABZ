/* GasCap Asset Tracker contains:
   CMWX1ZZABZ (STM32L082 and SX1276)
   LIS2DW12 accelerometer and BME280 environmental sensor
   CAM M8Q Concurrent GNSS engine
   MX25R6435FZAI 8 MByte SPI NOR flash memory

   Same basic design as Cricket asset tracker but in a more convenient form factor

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
#include "LIS2DW12.h"
#include "BME280.h"
#include "SPIFlash.h"
#include "CayenneLPP.h"
#include "I2Cdev.h"

// #define Serial Serial3  // use Serial port on Tag Connect if no USB
bool SerialDebug = true;

// Production Cricket 1
const char *appEui = "70B3D57ED000964D";
const char *appKey = "60170539954A1E2C8B519658F0CAFB52";
const char *devEui = "70B3D57ED004F377";

CayenneLPP myLPP(64);

#define I2C_BUS Wire // Define the I2C bus (Wire instance) you wish to use

I2Cdev i2c_0(&I2C_BUS); // Instantiate the I2Cdev object and point to the desired I2C bus

TimerMillis LoRaTimer;             // instantiate LoraWAN timer
TimerMillis NoMotionActivityTimer; // instantiate low-frequency timer
TimerMillis InMotionActivityTimer; // instantiate high-frequency timer
TimerMillis DataLoggerTimer;       // instantiate data logger timer

// Internal MCU and battery voltage monitor definitions
// GasCap pin assignments
#define myLed 10 // blue led
#define VBAT_en 2
#define USER_BTN 2
#define VBAT_sense A1

float VDDA, VBAT, VBUS, STM32L0Temp;
uint32_t UID[3] = {0, 0, 0};
char buffer[32];

// LIS2DW12 definitions
#define LIS2DW12_intPin1 3  // interrupt1 pin definitions, wake interrupt pin
#define LIS2DW12_intPin2 A4 // interrupt2 pin definitions, sleep interrupt pin

// Specify sensor parameters //
LPMODE lpMode = LIS2DW12_LP_MODE_1;  // choices are low power modes 1, 2, 3, or 4
MODE mode = LIS2DW12_MODE_LOW_POWER; // choices are low power, high performance, and one shot modes
ODR odr = LIS2DW12_ODR_12_5_1_6HZ;   //  1.6 Hz in lpMode, max is 200 Hz in LpMode
FS fs = LIS2DW12_FS_2G;              // choices are 2, 4, 8, or 16 g
BW_FILT bw = LIS2DW12_BW_FILT_ODR2;  // choices are ODR divided by 2, 4, 10, or 20
FIFOMODE fifoMode = BYPASS;          // capture 32 samples of data before wakeup event, about 2 secs at 25 Hz
bool lowNoise = false;               // low noise or lowest power

float aRes = 0;        // Sensor data scale in mg/LSB
int16_t accelCount[3]; // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;     // 8-bit signed temperature output
uint8_t rawTempCount;  // raw temperature output
float temperature;     // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;      // variables to hold latest sensor data values
float offset[3];       // holds accel bias offsets
float stress[3];       // holds results of the self test
uint8_t status = 0, wakeSource = 0, FIFOstatus = 0, numFIFOSamples = 0;

// Logic flags to keep track of device states
bool LIS2DW12_wake_flag = false;
bool LIS2DW12_sleep_flag = false;
bool InMotion = false;

LIS2DW12 LIS2DW12(&i2c_0); // instantiate LIS2DW12 class

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
uint8_t Posr = P_OSR_01, Hosr = H_OSR_01, Tosr = T_OSR_01, Mode = Sleep, IIRFilter = full, SBy = t_1000ms; // set pressure amd temperature output data rate

int32_t rawPress, rawTemp, rawHumidity, compTemp;                 // pressure, humidity, and temperature raw count output for BME280
uint32_t compHumidity, compPress;                                 // variables to hold compensated BME280 humidity and pressure values
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

BME280 BME280(&i2c_0); // instantiate BME280 class

// CAM M8Q GNSS configuration
#define GNSS_en 5      // enable for GNSS 3.0 V LDO
#define pps 4          // 1 Hz fix pulse
#define GNSS_backup A0 // RTC backup for MAX M8Q

GNSSLocation myLocation;
GNSSSatellites mySatellites;

volatile bool isTracking = false;

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

// SPI Flash: 64 MBit (8 MByte) SPI Flash 32,768, 256-byte pages
#define csPin 25 // SPI Flash chip select pin

uint8_t flash_id[3] = {0, 0, 0};
uint16_t page_number = 0;  // set the page number for flash page write
uint8_t sector_number = 0; // set the sector number for sector write
uint8_t flashPage[256];    // array to hold the data for flash page write
volatile bool logData = false;

SPIFlash SPIFlash(csPin);

void setup() 
{
  if (SerialDebug) 
  {
    Serial.begin(115200);
    Serial.println("Serial enabled!");
    delay(5000);
  }

  STM32L0.getUID(UID);
  if (SerialDebug) {Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX); }

  LoRaWAN.getDevEui(buffer, 18);
  if (SerialDebug) {Serial.print("STM32L0 Device EUI = "); Serial.println(buffer); }
  
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with blue led off, since active LOW

  pinMode(VBAT_en, OUTPUT);
  digitalWrite(VBAT_en, LOW);

  pinMode(VBAT_sense, INPUT);
  analogReadResolution(12);

  pinMode(GNSS_backup, OUTPUT);
  digitalWrite(GNSS_backup, HIGH);

  GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ); // Start GNSS
  while (GNSS.busy()) { } // wait for begin to complete

  GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GLONASS); // choose satellites
  while (GNSS.busy()) { } // wait for set to complete

  GNSS.setAntenna(GNSS.ANTENNA_INTERNAL); // GNSS.ANTENNA_INTERNAL
  while (GNSS.busy()) { } // wait for set to complete

  GNSS.enableWakeup(); // allow GNSS sleep and wakeup

  pinMode(LIS2DW12_intPin1, INPUT); // define LIS2DW12 wake and sleep interrupt pins as L082 inputs
  pinMode(LIS2DW12_intPin2, INPUT);

  /* initialize two wire bus */
  I2C_BUS.begin();          // Set master mode, default on SDA/SCL for STM32L0
  I2C_BUS.setClock(400000); // I2C frequency at 400 kHz
  delay(100);

  i2c_0.I2Cscan(); // should detect all I2C devices on the bus
  delay(100);

  // Read the LIS2DW12 Chip ID register, this is a good test of communication
  byte LIS2DW12_ChipID = LIS2DW12.getChipID();  // Read CHIP_ID register for LIS2DW12
  if(SerialDebug)
  {
    Serial.print("LIS2DW12 "); Serial.print("I AM "); Serial.print(LIS2DW12_ChipID, HEX); Serial.print(" I should be "); Serial.println(0x44, HEX);
    Serial.println(" ");
  }

  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte BME280_ChipID = BME280.getChipID();  // Read WHO_AM_I register for BME280
  if(SerialDebug)
  {
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(BME280_ChipID, HEX); Serial.print(" I should be "); Serial.println(0x60, HEX);
  Serial.println(" ");
  }

  delay(1000);

  if(LIS2DW12_ChipID == 0x44 && BME280_ChipID == 0x60) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   if (SerialDebug) { Serial.println("LIS2DW12 and BME280 are online..."); Serial.println(" "); }
   
    LIS2DW12.reset(); // software reset before initialization
    delay(100);

   LIS2DW12.selfTest(stress); // perform sensor self test
   if(SerialDebug)
   {
    Serial.print("x-axis self test = "); Serial.print(stress[0], 1); Serial.println("mg, should be between 70 and 1500 mg");
    Serial.print("y-axis self test = "); Serial.print(stress[1], 1); Serial.println("mg, should be between 70 and 1500 mg");
    Serial.print("z-axis self test = "); Serial.print(stress[2], 1); Serial.println("mg, should be between 70 and 1500 mg");
   }
   
    LIS2DW12.reset(); // software reset before initialization
    delay(100);

    aRes = 0.000244f * (1 << fs); // scale resolutions per LSB for the sensor at 14-bit data

    if (SerialDebug) Serial.println("hold flat and motionless for bias calibration");
    delay(5000);
    LIS2DW12.Compensation(fs, odr, mode, lpMode, bw, lowNoise, offset); // quickly estimate offset bias in normal mode
   if(SerialDebug)
   {
    Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
    Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
    Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg");
   }

    LIS2DW12.init(fs, odr, mode, lpMode, bw, lowNoise); // Initialize sensor in desired mode for application
    LIS2DW12.configureFIFO(fifoMode, 0x1F);             // 32 levels of data
    delay(100);                                         // let sensor settle

    // Configure the BME280 sensor
    BME280.reset(); // reset BME280 before initilization
    delay(100);

    BME280.init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy); // Initialize BME280 altimeter
    BME280.forced();                                     // get initial data sample, then go back to sleep
  }
  else {
    if (LIS2DW12_ChipID != 0x44 && SerialDebug)
      Serial.println(" LIS2DW12 not functioning!");
    if (BME280_ChipID != 0x60 && SerialDebug)
      Serial.println(" BME280 not functioning!");
  }

  pinMode(csPin, OUTPUT); // set SPI chip select as L082 output
  digitalWrite(csPin, HIGH);

  // check SPI Flash ID
  SPIFlash.init();              // start SPI
  SPIFlash.powerUp();           // MX25R6435FZAI defaults to power down state
  SPIFlash.getChipID(flash_id); // Verify SPI flash communication
  if (flash_id[0] == 0xC2 && flash_id[1] == 0x28 && flash_id[2] == 0x17 && SerialDebug)
  {
    Serial.println(" ");
    Serial.println("Found Macronix MX25R6435FZAI with Chip ID = 0xC2, 0x28, 0x17!");
    Serial.println(" ");
  }
  else {
    if (SerialDebug)
    {
      Serial.println(" ");
      Serial.println("no or unknown SPI flash!");
      Serial.println(" ");
    }
  }

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
  digitalWrite(VBAT_en, HIGH);
  VBAT = 1.27f * VDDA * ((float)analogRead(VBAT_sense)) / 4095.0f;
  digitalWrite(VBAT_en, LOW);
  STM32L0Temp = STM32L0.getTemperature();

  // Internal STM32L0 functions
  if (SerialDebug)
  {
    Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
    Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
    if(VBUS ==  1)  Serial.println("USB Connected!"); 
    Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
    Serial.println(" ");
  }

  // set alarm to update the RTC periodically
  RTC.setAlarmTime(12, 0, 0);
  RTC.enableAlarm(RTC.MATCH_SS); // alarm once per minute

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
  LoRaWAN.setSubBand(2); // 1 for SEMTECH, 2 for TTN servers

  LoRaWAN.joinOTAA(appEui, appKey, devEui);

  // Configure timers
  LoRaTimer.start(callbackLoRaTx, 60000, 600000);                        // ten minute period, delayed 1 minute
  NoMotionActivityTimer.start(callbackNoMotionActivity, 0, 3600000);     // low freq (one hour) timer, start right away
  InMotionActivityTimer.start(callbackInMotionActivity, 600000, 120000); // high freq (two minute) timer, ten minute delay
  DataLoggerTimer.start(callbackDataLogger, 60000, 60000);               // one minute period, delayed 1 minute

  attachInterrupt(LIS2DW12_intPin1, myinthandler1, RISING); // attach data ready/wake-up interrupt for INT1 pin output of LIS2DW12
  attachInterrupt(LIS2DW12_intPin2, myinthandler2, RISING); // attach no-motion          interrupt for INT2 pin output of LIS2DW12

  LIS2DW12.getStatus(); // read status of interrupts to clear
} /* end of setup */

/*
 * Everything in the main loop is based on interrupts, so that
 * if there has not been an interrupt event the STM32L082 should be in STOP mode
 */

void loop()
{
  /* LIS2DW12 wake detect*/
  if (LIS2DW12_wake_flag)
  {
    LIS2DW12_wake_flag = false; // clear the wake flag if wake event

    InMotion = true; // set motion state latch
    if (SerialDebug)
      Serial.println("** LIS2DW12 is awake! **");

    LIS2DW12.activateNoMotionInterrupt();
    attachInterrupt(LIS2DW12_intPin2, myinthandler2, RISING); // attach no-motion interrupt for INT2 pin output of LIS2DW12

    digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH); // toggle blue led when motion detected
  }                            /* end of LIS2DW12 wake detect */

  /* LIS2DW12 sleep detect*/ // Not needed for basic wake/GNSS on motion case
  if (LIS2DW12_sleep_flag)
  {
    LIS2DW12_sleep_flag = false; // clear the sleep flag
    InMotion = false;            // set motion state latch
    if (SerialDebug)
      Serial.println("** LIS2DW12 is asleep! **");

    detachInterrupt(LIS2DW12_intPin2);      // Detach the LIS2DW12 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L0
    LIS2DW12.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power

    digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH); // toggle blue led when no motion detected
  }                            /* end of LIS2DW12 sleep detect */

  /*GNSS*/
  if (GNSS.location(myLocation))
  {
    if (SerialDebug)
    {
      Serial.print("LOCATION: "); Serial.print(fixTypeString[myLocation.fixType()]);
    }

    if (myLocation.fixType() != GNSSLocation::TYPE_NONE)
    {
      Hour = myLocation.hours();
      Minute = myLocation.minutes();
      Second = myLocation.seconds();
      Year = myLocation.year();
      Month = myLocation.month();
      Day = myLocation.day();

      if (SerialDebug)
      {
        Serial.print(fixQualityString[myLocation.fixQuality()]);
        Serial.print(" ");
        Serial.print(myLocation.year());
        Serial.print("/");
        Serial.print(myLocation.month());
        Serial.print("/");
        Serial.print(myLocation.day());
        Serial.print(" ");
        if (myLocation.hours() <= 9) { Serial.print("0"); }
        Serial.print(myLocation.hours());
        Serial.print(":");
        if (myLocation.minutes() <= 9) { Serial.print("0"); }
        Serial.print(myLocation.minutes());
        Serial.print(":");
        if (myLocation.seconds() <= 9) { Serial.print("0"); }
        Serial.print(myLocation.seconds());
        Serial.print(".");
        if (myLocation.millis() <= 9) { Serial.print("0"); }
        if (myLocation.millis() <= 99) { Serial.print("0"); }
        Serial.print(myLocation.millis());
      }
      if (myLocation.leapSeconds() != GNSSLocation::LEAP_SECONDS_UNDEFINED)
      {
        if (SerialDebug)
        {
          Serial.print(" ");
          Serial.print(myLocation.leapSeconds());
        }
        if (!myLocation.fullyResolved())
        {
          if (SerialDebug)
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
        if (SerialDebug)
        {
          Serial.print(" LLA="); Serial.print(Lat, 7); Serial.print(",");
          Serial.print(Long, 7); Serial.print(","); Serial.print(Alt, 3);
          Serial.print(" EPE="); Serial.print(EPE, 3); Serial.print(","); Serial.print(myLocation.evpe(), 3);
          Serial.print(" SATELLITES="); Serial.print(myLocation.satellites());
          Serial.print(" DOP="); Serial.print(myLocation.hdop(), 2); Serial.print(","); Serial.print(myLocation.vdop(), 2);
          Serial.println();
        }
        // Put the CAM M8Q to sleep once 3D fix with sufficient accuracy is obtained
        if ((myLocation.fixType() != GNSSLocation::TYPE_2D) && (EPE <= 50.0f) && myLocation.fullyResolved())
        { // 10 is about as low as one should go, 50 is acceptable 
          if (!isTracking)
          {
            isTracking = true;

            if (SerialDebug)
              Serial.println("***GNSS go to sleep!***");
            GNSS.suspend();   // once we have a good 3D location fix put CAM M8Q to sleep
            callbackLoRaTx(); // update dashboard/backend via LoRaWAN
          }
        }
      }
    }

    if (SerialDebug)
      Serial.println();

  } /* end of GNSS Location handling */

  if (GNSS.satellites(mySatellites))
  {
    if (SerialDebug)
    {
      Serial.print("SATELLITES: ");
      Serial.println(mySatellites.count());
    }

    for (unsigned int index = 0; index < mySatellites.count(); index++)
    {
      unsigned int svid = mySatellites.svid(index);

      if ((svid >= 1) && (svid <= 32))
      {
        if (SerialDebug)
          Serial.print("    ");

        if (svid <= 9)
        {
          if (SerialDebug)
            Serial.print("  G");
          if (SerialDebug)
            Serial.print(svid);
        }
        else
        {
          if (SerialDebug)
            Serial.print(" G");
          if (SerialDebug)
            Serial.print(svid);
        }
      }
      else if ((svid >= 65) && (svid <= 96))
      {
        if (SerialDebug)
          Serial.print("    ");

        if ((svid - 64) <= 9)
        {
          if (SerialDebug)
            Serial.print("  R");
          if (SerialDebug)
            Serial.print(svid - 64);
        }
        else
        {
          if (SerialDebug)
            Serial.print(" R");
          if (SerialDebug)
            Serial.print(svid - 64);
        }
      }
      else if ((svid >= 120) && (svid <= 158))
      {
        if (SerialDebug)
        {
          Serial.print("    ");
          Serial.print("S");
          Serial.print(svid);
        }
      }
      else if ((svid >= 173) && (svid <= 182))
      {
        if (SerialDebug)
        {
          Serial.print("    ");
          Serial.print("  I");
          Serial.print(svid - 172);
        }
      }
      else if ((svid >= 193) && (svid <= 197))
      {
        if (SerialDebug)
        {
          Serial.print("    ");
          Serial.print("  Q");
          Serial.print(svid - 192);
        }
      }
      else if ((svid >= 211) && (svid <= 246))
      {
        if (SerialDebug)
          Serial.print("    ");

        if ((svid - 210) <= 9)
        {
          if (SerialDebug)
            Serial.print("  E");
          if (SerialDebug)
            Serial.print(svid - 210);
        }
        else
        {
          if (SerialDebug)
            Serial.print(" E");
          if (SerialDebug)
            Serial.print(svid - 210);
        }
      }
      else if (svid == 255)
      {
        if (SerialDebug)
          Serial.print("    ");
        if (SerialDebug)
          Serial.print("R???");
      }
      else
      {
        continue;
      }

      if (SerialDebug)
      {
        Serial.print(": SNR=");
        Serial.print(mySatellites.snr(index));
        Serial.print(", ELEVATION=");
        Serial.print(mySatellites.elevation(index));
        Serial.print(", AZIMUTH=");
        Serial.print(mySatellites.azimuth(index));
      }

      if (mySatellites.unhealthy(index))
      {
        if (SerialDebug) Serial.print(", UNHEALTHY");
      }

      if (mySatellites.almanac(index))
      {
        if (SerialDebug) Serial.print(", ALMANAC");
      }

      if (mySatellites.ephemeris(index))
      {
        if (SerialDebug) Serial.print(", EPHEMERIS");
      }

      if (mySatellites.autonomous(index))
      {
        if (SerialDebug) Serial.print(", AUTONOMOUS");
      }

      if (mySatellites.correction(index))
      {
        if (SerialDebug)
          Serial.print(", CORRECTION");
      }

      if (mySatellites.acquired(index))
      {
        if (SerialDebug) Serial.print(", ACQUIRED");
      }

      if (mySatellites.locked(index))
      {
        if (SerialDebug) Serial.print(", LOCKED");
      }

      if (mySatellites.navigating(index))
      {
        if (SerialDebug) Serial.print(", NAVIGATING");
      }

      if (SerialDebug) Serial.println();
    }

  } /* end of GNSS Satellites handling */

  /*RTC*/
  if (alarmFlag) { // update serial output
    alarmFlag = false;

    ax = ay = az = 0.0f;
    if (InMotion)
    {
      LIS2DW12.readAccelData(accelCount); // get 14-bit signed accel data

      // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0] * aRes - offset[0]; // get actual g value, this depends on scale being set
      ay = (float)accelCount[1] * aRes - offset[1];
      az = (float)accelCount[2] * aRes - offset[2];

      if (SerialDebug)
      {
        Serial.println(" ");
        Serial.print("ax = "); Serial.print((int)1000 * ax);
        Serial.print(" ay = "); Serial.print((int)1000 * ay);
        Serial.print(" az = "); Serial.print((int)1000 * az); Serial.println(" mg");
        Serial.println(" ");
      }
    }

    VDDA = STM32L0.getVDDA();
    digitalWrite(VBAT_en, HIGH);
    VBAT = 1.27f * VDDA * ((float)analogRead(VBAT_sense)) / 4095.0f;
    digitalWrite(VBAT_en, LOW);
    if (SerialDebug)
    {
      Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
    }

    tempCount = LIS2DW12.readTempData();      // Read the accel chip temperature adc values
    temperature = ((float)tempCount) + 25.0f; // 8-bit accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade
    if (SerialDebug)
    {
      Serial.print("Accel temperature is "); Serial.print(temperature, 1); Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    // BME280 Data
    BME280.forced(); // get one data sample, then go back to sleep

    rawTemp = BME280.readTemperature();
    compTemp = BME280.compensate_T(rawTemp);
    temperature_C = (float)compTemp / 100.0f;
    temperature_F = 9.0f * temperature_C / 5.0f + 32.0f;

    rawPress = BME280.readPressure();
    compPress = BME280.compensate_P(rawPress);
    pressure = (float)compPress / 25600.0f; // Pressure in mbar
    altitude = 145366.45f * (1.0f - powf((pressure / 1013.25f), 0.190284f));

    rawHumidity = BME280.readHumidity();
    compHumidity = BME280.compensate_H(rawHumidity);
    humidity = (float)compHumidity / 1024.0f; // Humidity in %RH

    if (SerialDebug)
    {
      Serial.println("BME280:");
      Serial.print("Altimeter temperature = ");
      Serial.print(temperature_C, 2);
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = ");
      Serial.print(temperature_F, 2);
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = ");
      Serial.print(pressure, 2);
      Serial.println(" mbar"); // pressure in millibar
      Serial.print("Altitude = ");
      Serial.print(altitude, 2);
      Serial.println(" feet");
      Serial.print("Altimeter humidity = ");
      Serial.print(humidity, 1);
      Serial.println(" %RH"); // pressure in millibar
      Serial.println(" ");
    }

    // Read RTC
    Serial.println("RTC:");
    RTC.getDate(day, month, year);
    RTC.getTime(hours, minutes, seconds, subSeconds);

    milliseconds = ((subSeconds >> 17) * 1000 + 16384) / 32768;

    if (SerialDebug)
    {
      Serial.print("RTC Time = ");
      if (hours < 10) { Serial.print("0"); } Serial.print(hours);
      Serial.print(":");
      if (minutes < 10) { Serial.print("0"); }
      Serial.print(minutes);
      Serial.print(":");
      if (seconds < 10) { Serial.print("0"); }
      Serial.print(seconds);
      Serial.print(".");
      if (milliseconds <= 9) { Serial.print("0"); } if (milliseconds <= 99) { Serial.print("0"); } Serial.print(milliseconds);
      Serial.println(" ");

      Serial.print("RTC Date = ");
      Serial.print(year); Serial.print(":"); Serial.print(month); Serial.print(":"); Serial.println(day);
      Serial.println();
    }

    digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH);

  } // end of RTC alarm section

  // log data to SPI flash
  if (logData)
  {
    logData = false;

    RTC.getDate(day, month, year);
    RTC.getTime(hours, minutes, seconds, subSeconds);
    accelCount[0] = 0;
    accelCount[1] = 0;
    accelCount[2] = 0;
    if (InMotion) { LIS2DW12.readAccelData(accelCount); } // read accel data
    rawTempCount = LIS2DW12.readRawTempData(); // Read the 8-bit accel chip temperature register

    // Store some data in flashPage array until we have a full 256-byte page
    uint8_t bps = 36; // bytes per sector such that 256 bytes per page = sectors per page x bps = 7 x 36 = 252 < 256
    if (sector_number < 7 && page_number < 0x7FFF) {                                                                   // 32,768 256-byte pages in a 8 MByte flash
      flashPage[sector_number * bps + 0] = (latOut & 0xFF000000) >> 24; // latitude in bytes
      flashPage[sector_number * bps + 1] = (latOut & 0x00FF0000) >> 16;
      flashPage[sector_number * bps + 2] = (latOut & 0x0000FF00) >> 8;
      flashPage[sector_number * bps + 3] = (latOut & 0x000000FF);
      flashPage[sector_number * bps + 4] = (longOut & 0xFF000000) >> 24; // longitude in bytes
      flashPage[sector_number * bps + 5] = (longOut & 0x00FF0000) >> 16;
      flashPage[sector_number * bps + 6] = (longOut & 0x0000FF00) >> 8;
      flashPage[sector_number * bps + 7] = (longOut & 0x000000FF);
      flashPage[sector_number * bps + 8] = ((uint16_t(Alt * 10.0f)) & 0xFF00) >> 8; // MSB GPS altitude
      flashPage[sector_number * bps + 9] = ((uint16_t(Alt * 10.0f)) & 0x00FF);      // LSB GPS altitude
      flashPage[sector_number * bps + 10] = (accelCount[0] & 0xFF00) >> 8;          // x-axis accel MSB
      flashPage[sector_number * bps + 11] = (accelCount[0] & 0x00FF);               // x-axis accel LSB
      flashPage[sector_number * bps + 12] = (accelCount[1] & 0xFF00) >> 8;          // y-axis accel MSB
      flashPage[sector_number * bps + 13] = (accelCount[1] & 0x00FF);               // y-axis accel LSB
      flashPage[sector_number * bps + 14] = (accelCount[2] & 0xFF00) >> 8;          // z-axis accel MSB
      flashPage[sector_number * bps + 15] = (accelCount[2] & 0x00FF);               // z-axis accel LSB
      flashPage[sector_number * bps + 16] = (compTemp & 0xFF000000) >> 24;
      flashPage[sector_number * bps + 17] = (compTemp & 0x00FF0000) >> 16;
      flashPage[sector_number * bps + 18] = (compTemp & 0x0000FF00) >> 8;
      flashPage[sector_number * bps + 19] = (compTemp & 0x000000FF);
      flashPage[sector_number * bps + 20] = (compHumidity & 0xFF000000) >> 24;
      flashPage[sector_number * bps + 21] = (compHumidity & 0x00FF0000) >> 16;
      flashPage[sector_number * bps + 22] = (compHumidity & 0x0000FF00) >> 8;
      flashPage[sector_number * bps + 23] = (compHumidity & 0x000000FF);
      flashPage[sector_number * bps + 24] = (compPress & 0xFF000000) >> 24;
      flashPage[sector_number * bps + 25] = (compPress & 0x00FF0000) >> 16;
      flashPage[sector_number * bps + 26] = (compPress & 0x0000FF00) >> 8;
      flashPage[sector_number * bps + 27] = (compPress & 0x000000FF);
      flashPage[sector_number * bps + 28] = seconds;
      flashPage[sector_number * bps + 29] = minutes;
      flashPage[sector_number * bps + 30] = hours;
      flashPage[sector_number * bps + 31] = day;
      flashPage[sector_number * bps + 32] = month;
      flashPage[sector_number * bps + 33] = year;
      flashPage[sector_number * bps + 34] = ((uint16_t(VBAT * 100.0f)) & 0xFF00) >> 8;
      flashPage[sector_number * bps + 35] = ((uint16_t(VBAT * 100.0f)) & 0x00FF);

      sector_number++;
    }

    // Once the page is full, write it to the SPI flash
    if (sector_number == 7 && page_number < 0x7FFF)
    {
      SPIFlash.powerUp();
      SPIFlash.flash_page_program(flashPage, page_number);
      if (SerialDebug) { Serial.print("***Wrote flash page: "); Serial.println(page_number); }
      digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH); // indicate when flash page is written
      sector_number = 0;
      page_number++;
      SPIFlash.powerDown(); // Put SPI flash into power down mode
    }

    //  if reached the last page stop logging
    if (page_number >= 0x7FFF)
    {
      if (SerialDebug) { Serial.println("Reached last page of SPI flash!"); Serial.println("Data logging stopped!"); }
    }

    digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH); // blink led when writing a flash page

  } // end of SPI flash logging

  STM32L0.stop(); // Enter STOP mode and wait for an interrupt

} /* end of loop*/

/* Useful functions */

void callbackLoRaTx(void)
{
  if (!LoRaWAN.joined())
  {
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
    delay(1000);
  }

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

void callbackDataLogger(void)
{
  logData = true;
  STM32L0.wakeup();
}

void callbackNoMotionActivity(void)
{
  GNSS.resume();
  isTracking = false;
}

void callbackInMotionActivity(void)
{
  if (InMotion)
  {
    InMotion = false;
    GNSS.resume();
    isTracking = false;
  }
}

void alarmMatch()
{
  alarmFlag = true;
  STM32L0.wakeup();
}

void myinthandler1()
{
  LIS2DW12_wake_flag = true;
  STM32L0.wakeup();
}

void myinthandler2()
{
  LIS2DW12_sleep_flag = true;
  STM32L0.wakeup();
}
