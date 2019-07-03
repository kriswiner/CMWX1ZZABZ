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
#include "VEML6030.h"
#include "MS5837.h"
#include "MS5803.h"
#include "LSM303AGR.h"
#include "LIS2MDL.h"
#include "SPIFlash.h"
#include "CayenneLPP.h"
#include "I2Cdev.h"

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

// Production Cricket 1
const char *appEui = "70B3D57ED000964D";
const char *appKey = "7DE66B18F7105B19A1427AFEB2514597";
const char *devEui = "373932325F376809";

CayenneLPP myLPP(64);

// Cricket pin assignments
#define myLed     10 // blue led 
#define myVBat_en  2 // enable VBat read
#define myVBat    A1 // VBat analog read pin

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

uint8_t Hour = 12, Minute = 0, Second = 0, Millisec, Year = 1, Month = 1, Day = 1;
uint8_t hours = 12, minutes = 0, seconds = 0, year = 1, month = 1, day = 1;
uint32_t subSeconds, milliseconds;
bool ppsFlag = false, firstSync = false, alarmFlag = true;
uint16_t count = 0, fixType = 0, fixQuality, latBytes[4], longBytes[4], tempBytes[4], pressBytes[4];
int32_t latOut, longOut;
float Long, Lat, Alt, EPE;
bool InMotion = true;

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


//LSM303AGR accel definitions
#define LSM303AGR_intPin1 A4  // interrupt1 pin definitions, data ready
#define LSM303AGR_intPin2 3   // interrupt2 pin definitions, significant motion

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      AODR_1Hz, AODR_10Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_400Hz 
*/ 
uint8_t Ascale = AFS_2G, AODR = AODR_100Hz; // assuming normal mode operation

float aRes;              // scale resolutions per LSB for the accel sensor
float accelBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel  
int16_t accelData[3], accTempData;  // Stores the 10-bit signed accel output if normal mode
float   accTemperature;             // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az;                   // variables to hold latest accel data values 

volatile bool newLSM303AGRData = false; // used for data ready interrupt handling
volatile bool newLSM303AGRactivity  = false; // used for activity interrupt handling

LSM303AGR LSM303AGR(&i2c_0); // instantiate LSM303AGR accel class

 
//LIS2MDL magnetometer definitions
#define LIS2MDL_intPin  A2 // interrupt for magnetometer data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
*/ 
uint8_t MODR = MODR_10Hz;

float mRes = 0.0015f;            // mag sensitivity
float magBias[3] = {0.0f, 0.0f, 0.0f}, magScale[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
int16_t magData[4];              // Stores the 16-bit signed sensor output
float magTemperature;            // Stores the real internal chip temperature in degrees Celsius
float mx, my, mz;                // variables to hold latest mag data values 
uint8_t LIS2MDLstatus;

volatile bool newLIS2MDLData = false; // LIS2MDL magnetometer interrupt flag

LIS2MDL LIS2MDL(&i2c_0); // instantiate LIS2MDL class


// Specify VEML6030 configuration parameters
// VEML6030 interrupt detects excursions below a low threshold abd above a high threshold
#define VEML6030Int A3

//Choices are:
// IT: (IT_25  25 ms, IT_50  50 ms,) IT_100 = 100 ms, IT_200 = 200 ms, IT_400 = 400 ms, IT_800 = 800 ms
// Gain: Gain_1x 1x gain, Gain_2x 2x gain, Gain_0_125 1/8 x gain, Gain_0_25 1/4 x gain
// Persistance: 0x00 = 1, 0x01 = 2, 0x02 = 4, 0x03 = 8 // Num,ber of times a threshold must be crossed to trigger an interrupt
// Power save Mode = 0x00, 0x01, 0x02, 0x03 higher the mode, longer time between data but lower the current usage
uint8_t IT = IT_100, Gain = Gain_1x, Persistance = 0x00, powerMode = 0x00;  // configuration variable

uint16_t ALSData = 0, WhiteData = 0, IntStatus = 0;
// lux/LSBit, ambient light sensitivity increases with integration time
float Sensitivity = 0.0288f/((float) (1 << IT) ); // for IT = 100 ms, 200 ms, 400 ms or 800 ms only
float ambientLight, whiteLight;
volatile bool VEML6030_flag = false;

VEML6030 VEML6030(&i2c_0);


// MS5837 configuration
// Specify sensor full scale
uint8_t MS5837_OSR = ADC_8192;     // set pressure amd temperature oversample rate

uint16_t MS5837_Pcal[8];         // calibration constants from MS5837 PROM registers
unsigned char MS5837_nCRC;       // calculated check sum to ensure PROM integrity
uint32_t MS5837_D1 = 0, MS5837_D2 = 0;  // raw MS5837 pressure and temperature data
double MS5837_dT, MS5837_OFFSET, MS5837_SENS, MS5837_TT2, MS5837_OFFSET2, MS5837_SENS2;  // First order and second order corrections for raw MS5837 temperature and pressure data
    
double MS5837_Temperature, MS5837_Pressure; // stores MS5837 pressures sensor pressure and temperature
float fluidDensity = 1029.0f; // kg/m^3 for seawater

MS5837 MS5837(&i2c_0); // instantiate MS5837 class

// MS5803 configuration
// Specify sensor full scale
uint8_t MS5803_OSR = ADC_4096;     // set pressure amd temperature oversample rate

uint16_t MS5803_Pcal[8];         // calibration constants from MS5803 PROM registers
unsigned char MS5803_nCRC;       // calculated check sum to ensure PROM integrity
uint32_t MS5803_D1 = 0, MS5803_D2 = 0;  // raw MS5803 pressure and temperature data
double MS5803_dT, MS5803_OFFSET, MS5803_SENS, MS5803_TT2, MS5803_OFFSET2, MS5803_SENS2;  // First order and second order corrections for raw MS5803 temperature and pressure data
    
double MS5803_Temperature, MS5803_Pressure; // stores MS5803 pressures sensor pressure and temperature

MS5803 MS5803(&i2c_0); // instantiate MS5803 class


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

  pinMode(myVBat_en, OUTPUT);
  digitalWrite(myVBat_en, LOW); // start with battery voltage monirot off
  pinMode(myVBat, INPUT);
  analogReadResolution(12);

  pinMode(LIS2MDL_intPin, INPUT);    // set up interrupt pins
  pinMode(LSM303AGR_intPin1, INPUT);
  pinMode(LSM303AGR_intPin2, INPUT);
  pinMode(VEML6030Int, INPUT);

  pinMode(GNSS_backup, OUTPUT);   // power for MAX M8Q RTC backup
  digitalWrite(GNSS_backup, HIGH);

  
  I2C_BUS.begin();                                      // Set master mode, default on SDA/SCL for STM32L4
  delay(1000);
  I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
  delay(1000);

  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                                      // should detect MS5803 at 0x76, MS5837 at 0x77, VEML6030 at 0x10, LIS2MDL at 0x1E, and LSM303AGR at 0x19
  delay(1000);


  GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ); // Start GNSS
  while (GNSS.busy()) { } // wait for begin to complete

  GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GLONASS); // choose satellites
  while (GNSS.busy()) { } // wait for set to complete

  GNSS.setAntenna(GNSS.ANTENNA_EXTERNAL); // GNSS.ANTENNA_INTERNAL or GNSS.ANTENNA_EXTERNAL
  while (GNSS.busy()) { } // wait for set to complete

  GNSS.enableWakeup();
  while (GNSS.busy()) { } // wait for set to complete

  pinMode(csPin, OUTPUT); // set SPI chip select as L082 output
  digitalWrite(csPin, HIGH);
 
  // check SPI Flash ID
  SPIFlash.init();      // start SPI
  SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state
  SPIFlash.getChipID(); // Verify SPI flash communication
  SPIFlash.powerDown(); // power down SPI flash

  // Read the LSM303AGR Chip ID register, this is a good test of communication
  Serial.println("LSM303AGR accel/gyro...");
  byte c = LSM303AGR.getChipID();  // Read CHIP_ID register for LSM303AGR
  Serial.print("LSM303AGR "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x33, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the LIS2MDL Chip ID register, this is a good test of communication
  Serial.println("LIS2MDL mag...");
  byte d = LIS2MDL.getChipID();  // Read CHIP_ID register for LSM303AGR
  Serial.print("LIS2MDL "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
  Serial.println(" ");
  delay(1000); 
  
  if(c == 0x33 && d == 0x40) // check if all I2C sensors have acknowledged
  {
   Serial.println("LSM303AGR and LIS2MDL are online..."); Serial.println(" ");
   
   digitalWrite(myLed, LOW); // turn on led during device initialization
 
   aRes = LSM303AGR.getAres(Ascale); // get sensor resolution, only need to do this once
   LSM303AGR.selfTest();
   LSM303AGR.reset();
   LSM303AGR.init(Ascale, AODR);

   LSM303AGR.offsetBias(accelBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   delay(1000); 

   mRes = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss); 
   LIS2MDL.selfTest();
   LIS2MDL.reset(); // software reset LIS2MDL to default registers  
   LIS2MDL.init(MODR);

   LIS2MDL.offsetBias(magBias, magScale);
   Serial.println("mag biases (mG)"); Serial.println(1000.0f * magBias[0]); Serial.println(1000.0f * magBias[1]); Serial.println(1000.0f * magBias[2]); 
   Serial.println("mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
   delay(2000); // add delay to see results before serial spew of data   
  }
  else 
  {
  if(c != 0x33) Serial.println(" LSM303AGR not functioning!"); // otherwise there is a problem somewhere
  if(d != 0x40) Serial.println(" LIS2MDL not functioning!");    
  while(1){};
  }

  // Set the RTC time
  SetDefaultRTC();

  VDDA = STM32L0.getVDDA();
  VBUS = STM32L0.getVBUS();
  digitalWrite(myVBat_en, HIGH);
  VBAT = 1.27f * VDDA * analogRead(myVBat) / 4096.0f;
  digitalWrite(myVBat_en, LOW);
  STM32L0Temp = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
  if(VBUS ==  1)  Serial.println("USB Connected!"); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
  Serial.println(" ");

  VEML6030.init(IT, Gain, Persistance); // initialize the VEML6030 ALS
  VEML6030.enablepowerSave(powerMode);
  VEML6030.setHighThreshold(0x0400); // set high threshold to 1024/65,536
  VEML6030.setLowThreshold(0x0008);  // set  low threshold to    8/65,536
  uint16_t HiThrs = VEML6030.getHighThreshold();
  uint16_t LoThrs = VEML6030.getLowThreshold();
  Serial.print("High Threshold is : 0x"); Serial.println(HiThrs, HEX);
  Serial.print("Lo Threshold is : 0x"); Serial.println(LoThrs, HEX);

  // Reset the MS5837 pressure sensor
  MS5837.Reset();
  delay(100);
  Serial.println("MS5837 pressure sensor reset...");
  // Read PROM data from MS5837 pressure sensor
  MS5837.PromRead(MS5837_Pcal);
  Serial.println("PROM data read:");
  Serial.print("C0 = "); Serial.println(MS5837_Pcal[0]);
  unsigned char MS5837_refCRC = MS5837_Pcal[0] >> 12;
  Serial.print("C1 = "); Serial.println(MS5837_Pcal[1]);
  Serial.print("C2 = "); Serial.println(MS5837_Pcal[2]);
  Serial.print("C3 = "); Serial.println(MS5837_Pcal[3]);
  Serial.print("C4 = "); Serial.println(MS5837_Pcal[4]);
  Serial.print("C5 = "); Serial.println(MS5837_Pcal[5]);
  Serial.print("C6 = "); Serial.println(MS5837_Pcal[6]);
  
  MS5837_nCRC = MS5837.checkCRC(MS5837_Pcal);  //calculate checksum to ensure integrity of MS5837 calibration data
  Serial.print("Checksum = "); Serial.print(MS5837_nCRC); Serial.print(" , should be "); Serial.println(MS5837_refCRC);  

  delay(1000);

  // Reset the MS5803 pressure sensor
  MS5803.Reset();
  delay(100);
  Serial.println("MS5803 pressure sensor reset...");
  // Read PROM data from MS5803 pressure sensor
  MS5803.PromRead(MS5803_Pcal);
  Serial.println("PROM data read:");
  Serial.print("C0 = "); Serial.println(MS5803_Pcal[0]);
  Serial.print("C1 = "); Serial.println(MS5803_Pcal[1]);
  Serial.print("C2 = "); Serial.println(MS5803_Pcal[2]);
  Serial.print("C3 = "); Serial.println(MS5803_Pcal[3]);
  Serial.print("C4 = "); Serial.println(MS5803_Pcal[4]);
  Serial.print("C5 = "); Serial.println(MS5803_Pcal[5]);
  Serial.print("C6 = "); Serial.println(MS5803_Pcal[6]);
  Serial.print("C7 = "); Serial.println(MS5803_Pcal[7]);
  unsigned char MS5803_refCRC = MS5803_Pcal[7] & 0x000F;;  
    
  MS5803_nCRC = MS5803.checkCRC(MS5803_Pcal);  //calculate checksum to ensure integrity of MS5803 calibration data
  Serial.print("Checksum = "); Serial.print(MS5803_nCRC); Serial.print(" , should be "); Serial.println(MS5803_refCRC); 
  delay(1000); 

  digitalWrite(myLed, HIGH); // turn off led when configuration successfully completed
  
  // set alarm to update the RTC periodically
  RTC.setAlarmTime(12, 0, 0);
  RTC.enableAlarm(RTC.MATCH_SS);  // alarm once per minute
//    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

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
    LoRaWAN.setSubBand(1); // 1 for MTCAP, 2 for TT gateways

    LoRaWAN.joinOTAA(appEui, appKey, devEui);

//    LoRaTimer.start(callbackLoRaTx, 300000, 600000);      //  10 minute period, delayed 5 minutes

//    NoMotionActivityTimer.start(callbackNoMotionActivity, 100000, 7200000);    // low  freq (two hours) timer
//    InMotionActivityTimer.start(callbackInMotionActivity, 100000, 7200000);    // high freq (one minute) timer

    // for testing
    LoRaTimer.start(callbackLoRaTx, 60000, 60000);      //  1 minute period, delayed 1 minute

    NoMotionActivityTimer.start(callbackNoMotionActivity, 0, 300000);          // low  freq (two hours) timer
    InMotionActivityTimer.start(callbackInMotionActivity, 100000,   60000);    // high freq (one minute) timer

    attachInterrupt(LSM303AGR_intPin1, myinthandler1, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR
    attachInterrupt(LSM303AGR_intPin2, myinthandler2, RISING);  // define no-motion activity interrupt for intPin2 output of LSM303AGR
    attachInterrupt(LIS2MDL_intPin ,   myinthandler3, RISING);  // define data ready interrupt for intPin  output of LIS2MDL
    attachInterrupt(VEML6030Int,       myinthandler4, FALLING); // threshold interrupt for light level

    LSM303AGR.readAccData(accelData); // INT1 cleared on any read
    LIS2MDL.readData(magData);  // read data register to clear interrupt before main loop
   
    /* end of setup */

}
/* 
 *  
 * Everything in the main loop is based on interrupts, so that 
 * if there has not been an interrupt event the STM32L082 should be in STOP mode
*/
void loop()
{
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
                    GNSS.suspend(); // once we have a good 3D location fix put MAX M8Q to sleep
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


// VEML6030 threshold interrupt handling
    if(VEML6030_flag)
    {
      VEML6030_flag = false;
      IntStatus = VEML6030.getIntStatus();
      if(IntStatus & 0x8000) Serial.println("Low Threshold Crossed!");
      if(IntStatus & 0x4000) Serial.println("High Threshold Crossed!");
   } /* end of VEML6030 interrupt handling */


  if(newLSM303AGRactivity)
  {
    newLSM303AGRactivity = false;
    Serial.println("no motion activity detected!");
  } // end of wake/sleep motion detection


     // If intPin goes high, either all data registers have new data
   if(newLSM303AGRData == true) {   // on interrupt, read data
      newLSM303AGRData = false;     // reset newData flag

     LSM303AGR.readAccData(accelData); // INT1 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     ax = (float)accelData[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)accelData[1]*aRes - accelBias[1];   
     az = (float)accelData[2]*aRes - accelBias[2];  
   }


    // If intPin goes high, either all data registers have new data
    if(newLIS2MDLData == true) {   // On interrupt, read data
      newLIS2MDLData = false;     // reset newData flag

     LIS2MDLstatus = LIS2MDL.status();
     
     if(LIS2MDLstatus & 0x08) // if all axes have new data ready
     {
      LIS2MDL.readData(magData);  
   
   // Now we'll calculate the accleration value into actual G's
     mx = (float)magData[0]*mRes - magBias[0];  // get actual G value 
     my = (float)magData[1]*mRes - magBias[1];   
     mz = (float)magData[2]*mRes - magBias[2]; 
     mx *= magScale[0];
     my *= magScale[1];
     mz *= magScale[2];  
     }
   }// end accel/mag sensor interrupt handling


  /*RTC*/
  if (alarmFlag) { // update serial output and log to SPI flash whenever there is an RTC alarm
    alarmFlag = false;
 
    // LIS2MDL/LSM303AGR data
    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("mx = "); Serial.print((int)1000*mx);  
    Serial.print(" my = "); Serial.print((int)1000*my); 
    Serial.print(" mz = "); Serial.print((int)1000*mz); Serial.println(" mG");
    }

    accTempData = LSM303AGR.readAccTempData();
    accTemperature = ((float) accTempData) + 25.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Accel temperature is ");  Serial.print(accTemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }
    
    // VEML6030 Data
    ALSData = VEML6030.getALSData();
    ambientLight = ((float)ALSData)*Sensitivity; // ALS in lux
    WhiteData = VEML6030.getWhiteData();
    whiteLight = ((float)WhiteData)*Sensitivity; // White light in lux
    Serial.print("VEML6030 ALS: "); Serial.print(ambientLight, 2); Serial.println(" lux");  
    Serial.print("VEML6030 White: "); Serial.print(whiteLight, 2); Serial.println(" lux"); Serial.println(" ");

   // MS5837 Data
    MS5837_D1 = MS5837.DataRead(ADC_D1, MS5837_OSR);  // get raw pressure value
    MS5837_D2 = MS5837.DataRead(ADC_D2, MS5837_OSR);  // get raw temperature value
    MS5837_dT = MS5837_D2 - MS5837_Pcal[5]*pow(2,8);    // calculate temperature difference from reference
    MS5837_OFFSET = MS5837_Pcal[2]*pow(2, 16) + MS5837_dT*MS5837_Pcal[4]/pow(2,7);
    MS5837_SENS = MS5837_Pcal[1]*pow(2,15) + MS5837_dT*MS5837_Pcal[3]/pow(2,8);
 
    MS5837_Temperature = (2000 + (MS5837_dT*MS5837_Pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
 
   // Second order corrections
    if(MS5837_Temperature > 20) 
    {
      MS5837_TT2 = 2*MS5837_dT*MS5837_dT/pow(2, 37); // correction for high temperatures
      MS5837_OFFSET2 = 1*(100*MS5837_Temperature - 2000)*(100*MS5837_Temperature - 2000)/16;
      MS5837_SENS2 = 0;
    }
    if(MS5837_Temperature < 20)                   // correction for low temperature
    {
      MS5837_TT2      = 3*MS5837_dT*MS5837_dT/pow(2, 33); 
      MS5837_OFFSET2 = 3*(100*MS5837_Temperature - 2000)*(100*MS5837_Temperature - 2000)/2;
      MS5837_SENS2   = 5*(100*MS5837_Temperature - 2000)*(100*MS5837_Temperature - 2000)/8;
    } 
    if(MS5837_Temperature < -15)                      // correction for very low temperature
    {
      MS5837_OFFSET2 = MS5837_OFFSET2 + 7*(100*MS5837_Temperature + 1500)*(100*MS5837_Temperature + 1500);
      MS5837_SENS2 = MS5837_SENS2 + 4*(100*MS5837_Temperature + 1500)*(100*MS5837_Temperature + 1500);
    }
    // End of second order corrections
 
     MS5837_Temperature = MS5837_Temperature - MS5837_TT2/100;
     MS5837_OFFSET = MS5837_OFFSET - MS5837_OFFSET2;
     MS5837_SENS = MS5837_SENS - MS5837_SENS2;
 
     MS5837_Pressure = (((MS5837_D1*MS5837_SENS)/pow(2, 21) - MS5837_OFFSET)/pow(2, 13))/10;  // Pressure in mbar or hPa

     float MS5837_altitude = 145366.45f*(1. - pow((MS5837_Pressure/1013.25f), 0.190284f));
     // Pressure in Pa, density in kg/mm^3,, gravity in m/s^2, i.e., SI units
     float MS5837_depth = (MS5837_Pressure*100.0f - 101300.0f)/(fluidDensity*9.80665f);
   
     if(SerialDebug) {
     Serial.print("MS5837 temperature value = "); Serial.print( (float)MS5837_Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
     Serial.print("MS5837 temperature value = "); Serial.print(9.*(float) MS5837_Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
     Serial.print("MS5837 pressure value = "); Serial.print((float) MS5837_Pressure, 2);  Serial.println(" mbar");// pressure in millibar
     Serial.print("MS5837 Altitude = "); Serial.print(MS5837_altitude, 2); Serial.println(" feet");
     Serial.print("MS5837 Depth = "); Serial.print(MS5837_depth, 2); Serial.println(" meters"); Serial.println(" ");
    }   

    // MS5803
    MS5803_D1 = MS5803.DataRead(ADC_D1, MS5803_OSR);  // get raw pressure value
    MS5803_D2 = MS5803.DataRead(ADC_D2, MS5803_OSR);  // get raw temperature value
    MS5803_dT = MS5803_D2 - MS5803_Pcal[5]*pow(2,8);    // calculate temperature difference from reference
    MS5803_OFFSET = MS5803_Pcal[2]*pow(2, 16) + MS5803_dT*MS5803_Pcal[4]/pow(2,7);
    MS5803_SENS = MS5803_Pcal[1]*pow(2,15) + MS5803_dT*MS5803_Pcal[3]/pow(2,8);
 
    MS5803_Temperature = (2000 + (MS5803_dT*MS5803_Pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
 
// Second order corrections
    if(MS5803_Temperature > 20) 
    {
      MS5803_TT2 = 0; // correction for high temperatures
      MS5803_OFFSET2 = 0;
      MS5803_SENS2 = 0;
    }
    if(MS5803_Temperature < 20)                   // correction for low temperature
    {
      MS5803_TT2     = MS5803_dT*MS5803_dT/pow(2, 31); 
      MS5803_OFFSET2 = 3*(100*MS5803_Temperature - 2000)*(100*MS5803_Temperature - 2000);
      MS5803_SENS2   = 7*(100*MS5803_Temperature - 2000)*(100*MS5803_Temperature - 2000)/8;
    } 
    if(MS5803_Temperature < -15)                      // correction for very low temperature
    {
      MS5803_OFFSET2 = MS5803_OFFSET2 + 7*(100*MS5803_Temperature + 1500)*(100*MS5803_Temperature + 1500);
      MS5803_SENS2 = MS5803_SENS2 + 2*(100*MS5803_Temperature + 1500)*(100*MS5803_Temperature + 1500);
    }
    // End of second order corrections
 
     MS5803_Temperature = MS5803_Temperature - MS5803_TT2/100;
     MS5803_OFFSET = MS5803_OFFSET - MS5803_OFFSET2;
     MS5803_SENS = MS5803_SENS - MS5803_SENS2;
 
     MS5803_Pressure = (((MS5803_D1*MS5803_SENS)/pow(2, 21) - MS5803_OFFSET)/pow(2, 15))/100;  // Pressure in mbar or hPa

     float MS5803_altitude = 145366.45f*(1. - pow((MS5803_Pressure/1013.25f), 0.190284f));
     // Pressure in Pa, density in kg/mm^3,, gravity in m/s^2, i.e., SI units
     float MS5803_depth = (MS5803_Pressure*100.0f - 101300.0f)/(fluidDensity*9.80665f);
   
     if(SerialDebug) {
     Serial.print("MS5803 temperature value = "); Serial.print( (float) MS5803_Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
     Serial.print("MS5803 temperature value = "); Serial.print(9.*(float) MS5803_Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
     Serial.print("MS5803 pressure value = "); Serial.print((float) MS5803_Pressure, 2);  Serial.println(" mbar");// pressure in millibar
     Serial.print("MS5803 Altitude = "); Serial.print(MS5803_altitude, 2); Serial.println(" feet");
     Serial.print("MS5803 Depth = "); Serial.print(MS5803_depth, 2); Serial.println(" meters"); Serial.println(" ");
    }   

    VDDA = STM32L0.getVDDA();
    digitalWrite(myVBat_en, HIGH);
    VBAT = 1.27f * VDDA * analogRead(myVBat) / 4096.0f;
    digitalWrite(myVBat_en, LOW);
    if(SerialDebug) {
      Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V"); Serial.println(" ");
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
          SPI.end();             // End SPI peripheral to save power in STOP mode     
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
        myLPP.addTemperature(1, MS5803_Temperature);
        myLPP.addBarometricPressure(2, MS5803_Pressure);
        myLPP.addTemperature(3, MS5837_Temperature);
        myLPP.addBarometricPressure(4, MS5837_Pressure);
        myLPP.addAnalogInput(5, VBAT);
        myLPP.addGPS(6, Lat, Long, Alt);

        LoRaWAN.sendPacket(myLPP.getBuffer(), myLPP.getSize());
     }
    
}


void callbackNoMotionActivity(void)
{
    GNSS.resume();
    isTracking = false;
    Serial.println("GNSS resumed!");
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
  newLSM303AGRData = true;
  STM32L0.wakeup();
}

void myinthandler2()
{
  newLSM303AGRactivity = true;
  STM32L0.wakeup();
}


void myinthandler3()
{
  newLIS2MDLData = true;
  STM32L0.wakeup();
}


void myinthandler4()
{
  VEML6030_flag = true;
  STM32L0.wakeup();
}


void alarmMatch()
{
  alarmFlag = true;
  STM32L0.wakeup();
}


void SetDefaultRTC()                                                                                 // Function sets the RTC to the FW build date-time...
{
  char Build_mo[3];
  String build_mo = "";

  Build_mo[0] = build_date[0];                                                                       // Convert month string to integer
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];
  for(uint8_t i=0; i<3; i++)
  {
    build_mo += Build_mo[i];
  }
  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;                                                                                       // Default to January if something goes wrong...
  }
  if(build_date[4] != 32)                                                                            // If the first digit of the date string is not a space
  {
    day   = (build_date[4] - 48)*10 + build_date[5]  - 48;                                           // Convert ASCII strings to integers; ASCII "0" = 48
  } else
  {
    day   = build_date[5]  - 48;
  }
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1]  - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4]  - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7]  - 48;
  RTC.setDay(day);                                                                                   // Set the date/time
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}


