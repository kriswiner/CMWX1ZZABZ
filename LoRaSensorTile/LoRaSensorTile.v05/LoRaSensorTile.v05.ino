/* 01/14/2018 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch is to operate the LoRa Sensor Tile (see https://hackaday.io/project/19649-stm32l4-sensor-tile) 
 with BME280 pressure/temperature/humidity sensor and BMA280 accelerometer hosted by an STM32L082 MCU. 
 The STM32L082 is embedded in Murata's CMWX1ZZABZ-078 module with a SX1276 LoRa radio modem for connectivity. 
 There is also a 16 MByte SPI flash for data logging and storage.

 Added capability of wake on any motion and sleep on no motion interrupts using BMA280 so activity can be tied to
 motion alerts of the device. This isa useful method of power management.

 Added CayenneLPP syntax to push data to mydevices.com dashboard.
 
 The sketch uses default SDA/SCL pins on the LoRa Sensor Tile.

 Library may be used freely and without limit with attribution.
*/
  
#include <Arduino.h>
#include <STM32L0.h>
#include "LoRaWAN.h"
#include "TimerMillis.h"
#include <RTC.h>
#include "BMA280.h"
#include "BME280.h"
#include "VEML6040.h"
#include "SPIFlashClass.h"
#include "CayenneLPP.h"

//LoraSensorTileNode2
const char *appEui = "70B3D57ED00093FB";
const char *appKey = "81457B0186BF3C67652AA075DD2A0DD3";
const char *devEui = "373932325c376b09";

CayenneLPP myLPP(64);

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 10          // blue led on LoRa Sensor Tile

uint8_t LoRaData[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

TimerMillis LoRaTimer;
TimerMillis NoMotionActivityTimer;  // instantiate low-frequency timer
TimerMillis InMotionActivityTimer;  // instantiate high-frequency timer

// battery voltage monitor definitions
float VDDA, VBAT, VBUS, Temperature;
uint32_t UID[3] = {0, 0, 0}; 
char buffer[32];

// BME280 definitions
/* Specify BME280 configuration
 *  Choices are:
 P_OSR_01, P_OSR_02, P_OSR_04, P_OSR_08, P_OSR_16    // pressure oversampling
 H_OSR_01, H_OSR_02, H_OSR_04, H_OSR_08, H_OSR_16    // humidity oversampling
 T_OSR_01, T_OSR_02, T_OSR_04, T_OSR_08, T_OSR_16    // temperature oversampling
 full, BW0_223ODR,BW0_092ODR, BW0_042ODR, BW0_021ODR // bandwidth at 0.021 x sample rate
 BME280Sleep, forced, forced2, normal //operation modes
 t_00_5ms = 0, t_62_5ms, t_125ms, t_250ms, t_500ms, t_1000ms, t_10ms, t_20ms // determines sample rate
 */
uint8_t Posr = P_OSR_01, Hosr = H_OSR_01, Tosr = T_OSR_01, Mode = Sleep, IIRFilter = full, SBy = t_1000ms;     // set pressure amd temperature output data rate

uint32_t rawPress, rawTemp, compHumidity, compTemp, compPress;    // pressure, humidity, and temperature raw count output for BME280
uint16_t rawHumidity;  // variables to hold raw BME280 humidity value
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

BME280 BME280; // instantiate BME280 class


// Specify VEML6040 Integration time
/*Choices are:
 IT_40  40 ms, IT_80  80 ms, IT_160  160 ms, IT_320  320 ms, IT_640  640 ms, IT_1280  1280 ms*/
uint8_t IT = IT_40;  // integration time variable
uint8_t ITime = 40;  // integration time in milliseconds
uint16_t RGBWData[4] = {0, 0, 0, 0};
float GSensitivity = 0.25168/((float) (1 << IT) ); // ambient light sensitivity increases with integration time
float ambientLight;

VEML6040 VEML6040;


// RTC set up
/* Change these values to set the current initial time */

uint8_t seconds = 0;
uint8_t minutes = 12;
uint8_t hours = 10;

/* Change these values to set the current initial date */

uint8_t day = 31;
uint8_t month = 1;
uint8_t year = 18;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = true;


//BMA280 definitions
#define BMA280_intPin1 2   // interrupt1 pin definitions
#define BMA280_intPin2 3   // interrupt2 pin definitions

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      BW_7_81Hz, BW_15_63Hz, BW_31_25Hz, BW_62_5Hz, BW_125Hz, BW_250Hz, BW_500Hz, BW_1000Hz
      normal_Mode, deepSuspend_Mode, lowPower_Mode, suspend_Mode
      sleep_0_5ms, sleep_1ms, sleep_2ms, sleep_4ms, sleep_6ms, sleep_10ms, sleep_25ms, sleep_50ms, sleep_100ms, sleep_500ms, sleep_1000ms
      low_power_Mode = lp_mode_1 or lp_mode_2
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


// 128 MBit (16 MByte) SPI Flash 65,536, 256-byte pages
#define csPin 25 // SPI Flash chip select pin (PH0)

uint16_t page_number = 0;     // set the page mumber for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write

SPIFlashClass SPIFlash(csPin);


void setup() {
    
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX); 

  LoRaWAN.getDevEui(buffer, 18);
  Serial.print("STM32L0 Device EUI = "); Serial.println(buffer); 
  
  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // start with led on
 
  Wire.begin(); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);

  BME280.I2Cscan(); // should detect BME280 at 0x77, BMA280 at 0x18, and VEML6040 at 0x10
  delay(1000);
    
  // Set the time
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);

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
  BMA280.resetBMA280();                                              // software reset before self-test
  delay(100);                                                        // give some time to read the screen
  BMA280.selfTestBMA280();                                           // perform sensor self test
  BMA280.resetBMA280();                                              // software reset before initialization
  delay(1000);                                                       // give some time to read the screen
  BMA280.initBMA280(Ascale, BW, normal_Mode, sleep_dur);             // initialize sensor in normal mode for calibration
  BMA280.fastCompensationBMA280();                                   // quickly estimate offset bias
  BMA280.initBMA280_MotionManager(Ascale, BW, power_Mode, sleep_dur, // Initialize sensor in desired mode for application
                                    low_power_Mode, motion_threshold);

  BME280.resetBME280();                                              // reset BME280 before initilization
  delay(100);

  BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy);         // Initialize BME280 altimeter
  BME280.BME280forced();                                             // get initial data sample, then go back to sleep

  digitalWrite(myLed, HIGH); // turn off led when succwessfully through sensor configuration
  
  }
  else 
  {
  if(c != 0xFB) Serial.println(" BMA280 not functioning!");
  if(d != 0x60) Serial.println(" BME280 not functioning!");     
  }

    VEML6040.enableVEML6040(IT); // initalize VEML6040 sensor
    delay(150);  

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
  
    // check SPI Flash ID
    SPIFlash.init();
    SPIFlash.powerUp();
    SPIFlash.getChipID(); 
  
    // set alarm to update the RTC every second
    RTC.setAlarmTime(0, 0, 0);
    RTC.enableAlarm(RTC.MATCH_SS);  // alarm once per minute
//    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
    // set up interrupts
    RTC.attachInterrupt(alarmMatch);

    attachInterrupt(BMA280_intPin1, myinthandler1, RISING);  // define interrupt for INT1 pin output of BMA280
    attachInterrupt(BMA280_intPin2, myinthandler2, RISING);  // define interrupt for INT2 pin output of BMA280

  // Configure LoRaWAN connection
  /*
    - Asia      AS923
    - Australia AU915
    - Europe    EU868
    - India     IN865
    - Korea     KR920
    - US        US915 (64 + 8 channels)
   */
    LoRaWAN.begin(US915);
    LoRaWAN.setADR(false);
    LoRaWAN.setDataRate(1);
    LoRaWAN.setTxPower(10);
    LoRaWAN.setSubBand(1); // 1 for MTCAP and 2 for TTN gateway

    LoRaWAN.joinOTAA(appEui, appKey, devEui);

    LoRaTimer.start(callbackLoRaTx,  60000, 360000);  // 6 minute period, 1 minute delay

    NoMotionActivityTimer.start(callbackNoMotionActivity, 100000, 600000);   //  low freq (ten minute) timer
    InMotionActivityTimer.start(callbackInMotionActivity, 100000,  60000);   // high freq (one minute) timer
    
    /* end of setup */

}

void loop() {

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
    }


    if(ActivityOn){ // if ActivityOn flag it true, carry out activity until done condition reached
       InMotion = false;  // turn off ImMotion flag until next motion event is detected

      // Begin activity
      digitalWrite(myLed, LOW); 
      delay(100); 
      
      // The activity can be carried out over many loop cycles ending when some threshold criterion is reached
      // The activity could be send LoRaWAN data, activate a relay, take a picture with a camera, etc
      // Here we just blink the led once and end
      
      // End activity
      ActivityOn = false;
      digitalWrite(myLed, HIGH);
    }

   
    if(alarmFlag)  { // update RTC output (serial display) whenever the RTC alarm condition is achieved
       alarmFlag = false;

/*    
 *     tempCount = BMA280.readBMA280TempData();  // Read the accel chip temperature adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip temperature in degrees Centigrade
 
    if(SerialDebug) {
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    // Print temperature in degrees Centigrade      
    Serial.print("Accel temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        
    }
*/
    // BME280 Data
    BME280.BME280forced();  // get one data sample, then go back to sleep
    
    rawTemp =  BME280.readBME280Temperature();
    compTemp = BME280.BME280_compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    rawPress =  BME280.readBME280Pressure();
    compPress = BME280.BME280_compensate_P(rawPress);
    pressure = (float) compPress/25600.f; // Pressure in mbar
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

    // VEML6040 Data
    VEML6040.enableVEML6040(IT); // enable VEML6040 sensor
    delay(ITime);  // wait for integration of light sensor data
    VEML6040.getRGBWdata(RGBWData); // read light sensor data
    VEML6040.disableVEML6040(IT); // disable VEML6040 sensor
 
    if(SerialDebug){ 
    Serial.print("Red raw counts = ");   Serial.println(RGBWData[0]);
    Serial.print("Green raw counts = "); Serial.println(RGBWData[1]);
    Serial.print("Blue raw counts = ");  Serial.println(RGBWData[2]);
    Serial.print("White raw counts = "); Serial.println(RGBWData[3]);
    Serial.print("Inferred IR raw counts = "); Serial.println(RGBWData[3] - RGBWData[0] - RGBWData[1] - RGBWData[2]);
    Serial.println("  ");
 
    Serial.print("Red   light power density = "); Serial.print((float)RGBWData[0]/96.0f, 2); Serial.println(" microWatt/cm^2");
    Serial.print("Green light power density = "); Serial.print((float)RGBWData[1]/74.0f, 2); Serial.println(" microWatt/cm^2");
    Serial.print("Blue  light power density = "); Serial.print((float)RGBWData[2]/56.0f, 2); Serial.println(" microWatt/cm^2");
    Serial.println("  ");

    ambientLight = (float)RGBWData[1]*GSensitivity;
    Serial.print("Ambient light intensity = "); Serial.print(ambientLight, 2); Serial.println(" lux");
    Serial.println("  ");

    // Empirical estimation of the correlated color temperature CCT:
    // see https://www.vishay.com/docs/84331/designingveml6040.pdf
    float temp = ( (float) (RGBWData[0] - RGBWData[2])/(float) RGBWData[1] );
    float CCT = 4278.6f*pow(temp, -1.2455f) + 0.5f;

    Serial.print("Correlated Color Temperature = "); Serial.print(CCT); Serial.println(" Kelvin");
    Serial.println("  ");
    }
    
    // Read RTC
    Day     = RTC.getDay();
    Month   = RTC.getMonth();
    Year    = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    
    if(SerialDebug){ 
    Serial.println("RTC:");
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");
    }

//  VDDA = STM32L0.getVDDA();
//  VBUS = STM32L0.getVBUS();
  VBAT = STM32L0.getVBAT();
//  Temperature = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
//  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
//  if(VBUS ==  1)  Serial.println("USB Connected!"); 
//  Serial.print("STM32L0 MCU Temperature = "); Serial.println(Temperature, 2);

    // Highest page number is 0xFFFF = 65535 for 128 Mbit flash
    // store some data to the SPI flash
    // each page holds 9 sectors of 28 bytes each
      if(sector_number < 9 && page_number < 0xFFFF) {
      flashPage[sector_number*28 + 0] =  (compTemp & 0xFF000000) >> 24;
      flashPage[sector_number*28 + 1] =  (compTemp & 0x00FF0000) >> 16;
      flashPage[sector_number*28 + 2] =  (compTemp & 0x0000FF00) >> 8;
      flashPage[sector_number*28 + 3] =  (compTemp & 0x000000FF);
      flashPage[sector_number*28 + 4] =  (compHumidity & 0xFF000000) >> 24;
      flashPage[sector_number*28 + 5] =  (compHumidity & 0x00FF0000) >> 16;
      flashPage[sector_number*28 + 6] =  (compHumidity & 0x0000FF00) >> 8;
      flashPage[sector_number*28 + 7] =  (compHumidity & 0x000000FF);
      flashPage[sector_number*28 + 8] =  (compPress & 0xFF000000) >> 24;
      flashPage[sector_number*28 + 9] =  (compPress & 0x00FF0000) >> 16;
      flashPage[sector_number*28 + 10] = (compPress & 0x0000FF00) >> 8;
      flashPage[sector_number*28 + 11] = (compPress & 0x000000FF);
      flashPage[sector_number*28 + 12] = Seconds;
      flashPage[sector_number*28 + 13] = Minutes;
      flashPage[sector_number*28 + 14] = Hours;
      flashPage[sector_number*28 + 15] = Day;
      flashPage[sector_number*28 + 16] = Month;
      flashPage[sector_number*28 + 17] = Year;
      flashPage[sector_number*28 + 18] = ( (uint16_t (VBAT * 100.f)) & 0xFF00) >> 8;
      flashPage[sector_number*28 + 19] = ( (uint16_t (VBAT * 100.f)) & 0x00FF);
      flashPage[sector_number*28 + 20] = (RGBWData[0] & 0xFF00) >> 8;
      flashPage[sector_number*28 + 21] = (RGBWData[0] & 0x00FF);
      flashPage[sector_number*28 + 22] = (RGBWData[1] & 0xFF00) >> 8;
      flashPage[sector_number*28 + 23] = (RGBWData[1] & 0x00FF);
      flashPage[sector_number*28 + 24] = (RGBWData[2] & 0xFF00) >> 8;
      flashPage[sector_number*28 + 25] = (RGBWData[2] & 0x00FF);
      flashPage[sector_number*28 + 26] = (RGBWData[3] & 0xFF00) >> 8;
      flashPage[sector_number*28 + 27] = (RGBWData[3] & 0x00FF);
      sector_number++;
    }
    else if(sector_number == 9 && page_number < 0xFFFF)
    {
      SPIFlash.powerUp();
      SPIFlash.flash_page_program(flashPage, page_number);
      Serial.print("Wrote flash page: "); Serial.println(page_number);
      digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH); // indicate when flash page is written
      sector_number = 0;
      page_number++;
    }  
    else
    {
      Serial.println("Reached last page of SPI flash!"); Serial.println("Data logging stopped!");
    }

      digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH);
    
    } // end of alarm section


    SPIFlash.powerDown();  // Put SPI flash into power down mode
    SPI.end();             // End SPI peripheral to save power in STOP mode
    STM32L0.stop();        // Enter STOP mode and wait for an interrupt
    SPI.begin();           // When exiting STOP mode, re-enable the SPI peripheral
   
}

/* Useful functions */

void callbackLoRaTx(void)
{     
 /*     
      // Send some data via LoRaWAN
      LoRaData[0]  = (uint16_t(temperature_C*100.0) & 0xFF00) >> 8;
      LoRaData[1]  =  uint16_t(temperature_C*100.0) & 0x00FF;
      LoRaData[2] =  (uint16_t(pressure*10.0      ) & 0xFF00) >> 8;   
      LoRaData[3] =   uint16_t(pressure*10.0      ) & 0x00FF;         
      LoRaData[4] =  (uint16_t(humidity*100.0     ) & 0xFF00) >> 8;
      LoRaData[5] =   uint16_t(humidity*100.0     ) & 0x00FF;
      LoRaData[6] =  ( RGBWData[1]  & 0xFF00) >> 8;
      LoRaData[7] =    RGBWData[1]  & 0x00FF;
      LoRaData[8] =   uint8_t(VBAT*50.0); // maximum should be 4.2 * 50 = 210
  */

    if (!LoRaWAN.busy() && LoRaWAN.joined())
     {
/*      LoRaWAN.beginPacket(3);
        LoRaWAN.write(LoRaData, sizeof(LoRaData));
        LoRaWAN.endPacket();
*/
        myLPP.reset();
        myLPP.addTemperature(1, temperature_C);
        myLPP.addRelativeHumidity(2, humidity);
        myLPP.addBarometricPressure(3, pressure);
        myLPP.addLuminosity(4, ambientLight);
        LoRaWAN.sendPacket(myLPP.getBuffer(), myLPP.getSize());
     }
    
}

void callbackNoMotionActivity(void)
{
  ActivityOn = true;
}


void callbackInMotionActivity(void)
{
  if(InMotion)
  {
    InMotion = false;
    ActivityOn = true;
  }
}


 void alarmMatch()
{
  alarmFlag = true; // Just set flag when interrupt received, don't try reading data in an interrupt handler
  STM32L0.wakeup();
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

