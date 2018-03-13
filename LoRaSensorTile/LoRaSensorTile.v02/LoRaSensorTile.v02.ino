/* 01/14/2018 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch is to operate the LoRa Sensor Tile (see https://hackaday.io/project/19649-stm32l4-sensor-tile) 
 with BME280 pressure/temperature/humidity sensor and BMA280 accelerometer hosted by an STM32L082 MCU. 
 The STM32L082 is embedded in Murata's CMWX1ZZABZ-078 module with a SX1276 LoRa radio modem for connectivity. 
 There is also an 8 MByte SPI flash for data logging and storage.
 
 The sketch uses default SDA/SCL pins on the LoRa Sensor Tile.

 Library may be used freely and without limit with attribution.
*/
  
#include <Arduino.h>
#include <STM32L0.h>
#include "LoRaWAN.h"
#include <RTC.h>
#include "BMA280.h"
#include "BME280.h"
#include "VEML6040.h"
#include "SPIFlashClass.h"

const char *appEui = "70B3D57ED00093FB";
const char *appKey = "D3FC12149485EA7B9A4D8F5B6484830E";
const char *devEui = "9473730323239372";

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 10          // blue led on LoRa Sensor Tile
#define myBat A1          // 270K/1000K voltage divider on A1 ADC

uint8_t LoRaData[7] = {0, 0, 0, 0, 0, 0, 0};

// battery voltage monitor definitions
float VDDA, VBAT, Temperature;
uint32_t UID[3] = {0, 0, 0}; 
uint16_t rawVbat = 0;
uint16_t count = 0;

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


// Specify VEML6040 Integration time
/*Choices are:
 IT_40  40 ms, IT_80 80 ms, IT_160  160 ms, IT_320  320 ms, IT_640 640 ms, IT_1280 1280 ms*/
uint8_t IT = IT_40;  // integration time variable
uint8_t ITime = 40;  // integration time in milliseconds
uint16_t RGBWData[4] = {0, 0, 0, 0};
float GSensitivity = 0.25168/((float) (1 << IT) ); // ambient light sensitivity increases with integration time
float redLight, greenLight, blueLight, ambientLight;

VEML6040 VEML6040;


// RTC set up
/* Change these values to set the current initial time */

uint8_t seconds = 0;
uint8_t minutes = 06;
uint8_t hours = 11;

/* Change these values to set the current initial date */

uint8_t day = 14;
uint8_t month = 1;
uint8_t year = 18;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = false;


//BMA280 definitions
#define BMA280_intPin1 2   // interrupt1 pin definitions
#define BMA280_intPin2 3   // interrupt2 pin definitions

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      BW_7_81Hz, BW_15_63Hz, BW_31_25Hz, BW_62_5Hz, BW_125Hz, BW_250Hz, BW_500Hz, BW_1000Hz
      normal_Mode, deepSuspend_Mode, lowPower_Mode, suspend_Mode
      sleep_0_5ms, sleep_1ms, sleep_2ms, sleep_4ms, sleep_6ms, sleep_10ms, sleep_25ms, sleep_50ms, sleep_100ms, sleep_500ms, sleep_1000ms
*/ 
uint8_t Ascale = AFS_2G, BW = BW_250Hz, power_Mode = lowPower_Mode, sleep_dur = sleep_1000ms, tapStatus, tapType;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 

bool newBMA280Data = false;
bool newBMA280Tap  = false;

BMA280 BMA280(BMA280_intPin1, BMA280_intPin2); // instantiate BMA280 class


// 128 MBit (16 MByte) SPI Flash 65, 536, 256-byte pages
#define csPin 21 // SPI Flash chip select pin (PH0)

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

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // start with led on

  pinMode(myBat, INPUT);
  analogReadResolution(12);  // set ADC resolution to 12 bit
 
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
  BMA280.selfTestBMA280();                                           // perform sensor self test
  BMA280.resetBMA280();                                              // software reset before initialization
  delay(1000);                                                       // give some time to read the screen
  BMA280.initBMA280(Ascale, BW, normal_Mode, sleep_dur);             // initialize sensor in normal mode for calibration
  BMA280.fastCompensationBMA280();                                   // quickly estimate offset bias
  BMA280.initBMA280(Ascale, BW, power_Mode, sleep_dur);              // initialize sensor for data acquisition

  BME280.resetBME280();                                              // reset BME280 before initilization
  delay(100);

  BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy);         // Initialize BME280 altimeter

  }
  else 
  {
  if(c != 0xFB) Serial.println(" BMA280 not functioning!");
  if(d != 0x60) Serial.println(" BME280 not functioning!");     
  }

    VEML6040.enableVEML6040(IT); // initalize VEML6040 sensor
    delay(150);  
    
    SPIFlash.init();
    SPIFlash.getChipID(); // get SPI Flash chip ID
  
    // set alarm to update the RTC every second
    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
    // set up interrupts
    RTC.attachInterrupt(alarmMatch);

    attachInterrupt(BMA280_intPin1, myinthandler1, RISING);  // define interrupt for INT1 pin output of BMA280
    attachInterrupt(BMA280_intPin2, myinthandler2, RISING);  // define interrupt for INT2 pin output of BMA280

  // Configuree LoRaWAN connection
  /*
    - Asia     REGION_AS923
    - Australia REGION_AU915
    - Europe    REGION_EU868
    - India     REGION_IN865
    - Korea     REGION_KR920
    - US        REGION_US915 (64 + 8 channels)
   */
    LoRaWAN.begin(US915);
    LoRaWAN.setAdrEnable(false);
    LoRaWAN.setDataRate(1);
    LoRaWAN.setTxPower(10);
    LoRaWAN.setSubBand(2); // for TT 

    LoRaWAN.joinOTAA(appEui, appKey, devEui);
    
    /* end of setup */

}

void loop() {

    // BMA280 acceleration
    if(newBMA280Data == true) {  // On interrupt, read data
     newBMA280Data = false;  // reset newData flag

     BMA280.readBMA280AccelData(accelCount); // get 14-bit signed accel data
     
     // Now we'll calculate the accleration value into actual g's
     ax = (float)accelCount[0]*aRes/4.0f;  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes/4.0f;   
     az = (float)accelCount[2]*aRes/4.0f;  
    }


    // Check for BMA280 taps
    if(newBMA280Tap == true) {  // On interrupt, identify tap
      newBMA280Tap = false;
 
      tapType = BMA280.getTapType();
      if(tapType & 0x20) Serial.println("Single tap detected!");
      if(tapType & 0x10) Serial.println("Double tap detected!"); 
      
      tapStatus = BMA280.getTapStatus();  // Read tap status register

      if(tapStatus & 0x80) {
        Serial.println("Tap is negative");
        }
      else {
        Serial.println("Tap is positive");
      }

       if(tapStatus & 0x40) {
        Serial.println("Tap is on x");
       }
       if(tapStatus & 0x20) {
        Serial.println("Tap is on y");
       }
       if(tapStatus & 0x10) {
        Serial.println("Tap is on z");
       }      
    }     

   
    if(alarmFlag)  { // update RTC output (serial display) whenever the RTC alarm condition is achieved
       alarmFlag = false;
       count++;

    if(SerialDebug) {
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    }

    tempCount = BMA280.readBMA280TempData();  // Read the accel chip temperature adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial.print("Accel temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        

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

    Serial.print("Ambient light intensity = "); Serial.print((float)RGBWData[1]*GSensitivity, 2); Serial.println(" lux");
    Serial.println("  ");

    // Empirical estimation of the correlated color temperature CCT:
    // see https://www.vishay.com/docs/84331/designingveml6040.pdf
    float temp = ( (float) (RGBWData[0] - RGBWData[2])/(float) RGBWData[1] );
    float CCT = 4278.6f*pow(temp, -1.2455f) + 0.5f;

    Serial.print("Correlated Color Temperature = "); Serial.print(CCT); Serial.println(" Kelvin");
    Serial.println("  ");
    
    // Read RTC
    Serial.println("RTC:");
    Day = RTC.getDay();
    Month = RTC.getMonth();
    Year = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");

    rawVbat = analogReadEx(myBat, 40000); // set sampling time to 40 milliseconds
    Serial.print("rawVbat = "); Serial.println(rawVbat); 
    VBAT = (1270.0f/1000.0f) * 3.30f * ((float)rawVbat)/4095.0f;
    Serial.print("VBAT = "); Serial.println(VBAT, 2); 

    // Highest page number is 0xFFFF = 65535 for 128 Mbit flash
    // store some data to the SPI flash
    // each page holds 7 sectors of 36 bytes each
      if(sector_number < 7 && page_number < 0xFFFF) {
      flashPage[sector_number*36 + 0]  =  tempCount;                     // Accel chip temperature
      flashPage[sector_number*36 + 1]  = (accelCount[0] & 0xFF00) >> 8;  // MSB x-axis accel
      flashPage[sector_number*36 + 2]  =  accelCount[0] & 0x00FF;        // LSB x-axis accel
      flashPage[sector_number*36 + 3]  = (accelCount[1] & 0xFF00) >> 8;  // MSB y-axis accel
      flashPage[sector_number*36 + 4]  =  accelCount[1] & 0x00FF;        // LSB y-axis accel
      flashPage[sector_number*36 + 5]  = (accelCount[2] & 0xFF00) >> 8;  // MSB z-axis accel
      flashPage[sector_number*36 + 6]  =  accelCount[2] & 0x00FF;        // LSB z-axis accel
      flashPage[sector_number*36 + 7] =  (compTemp & 0xFF000000) >> 24;
      flashPage[sector_number*36 + 8] =  (compTemp & 0x00FF0000) >> 16;
      flashPage[sector_number*36 + 9] =  (compTemp & 0x0000FF00) >> 8;
      flashPage[sector_number*36 + 10] = (compTemp & 0x000000FF);
      flashPage[sector_number*36 + 11] = (compHumidity & 0xFF000000) >> 24;
      flashPage[sector_number*36 + 12] = (compHumidity & 0x00FF0000) >> 16;
      flashPage[sector_number*36 + 13] = (compHumidity & 0x0000FF00) >> 8;
      flashPage[sector_number*36 + 14] = (compHumidity & 0x000000FF);
      flashPage[sector_number*36 + 15] = (compPress & 0xFF000000) >> 24;
      flashPage[sector_number*36 + 16] = (compPress & 0x00FF0000) >> 16;
      flashPage[sector_number*36 + 17] = (compPress & 0x0000FF00) >> 8;
      flashPage[sector_number*36 + 18] = (compPress & 0x000000FF);
      flashPage[sector_number*36 + 19] = Seconds;
      flashPage[sector_number*36 + 20] = Minutes;
      flashPage[sector_number*36 + 21] = Hours;
      flashPage[sector_number*36 + 22] = Day;
      flashPage[sector_number*36 + 23] = Month;
      flashPage[sector_number*36 + 24] = Year;
      flashPage[sector_number*36 + 25] = (rawVbat & 0xFF00) >> 8;
      flashPage[sector_number*36 + 26] = (rawVbat & 0x00FF);
      flashPage[sector_number*36 + 27] = (RGBWData[0] & 0xFF00) >> 8;
      flashPage[sector_number*36 + 28] = (RGBWData[0] & 0x00FF);
      flashPage[sector_number*36 + 29] = (RGBWData[1] & 0xFF00) >> 8;
      flashPage[sector_number*36 + 30] = (RGBWData[1] & 0x00FF);
      flashPage[sector_number*36 + 31] = (RGBWData[2] & 0xFF00) >> 8;
      flashPage[sector_number*36 + 32] = (RGBWData[2] & 0x00FF);
      flashPage[sector_number*36 + 33] = (RGBWData[3] & 0xFF00) >> 8;
      flashPage[sector_number*36 + 34] = (RGBWData[3] & 0x00FF);
      sector_number++;
    }
    else if(sector_number == 7 && page_number < 0xFFFF)
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

    if(count > 60)  // send LoRa data periodically
    {
      
      // Send some data via LoRaWAN
      LoRaData[0]  = (uint16_t(temperature_C*100.0) & 0xFF00) >> 8;
      LoRaData[1]  =  uint16_t(temperature_C*100.0) & 0x00FF;
      LoRaData[2] =  (uint16_t(pressure*10.0      ) & 0xFF00) >> 8;   
      LoRaData[3] =   uint16_t(pressure*10.0      ) & 0x00FF;         
      LoRaData[4] =  (uint16_t(humidity*100.0     ) & 0xFF00) >> 8;
      LoRaData[5] =   uint16_t(humidity*100.0     ) & 0x00FF;
      LoRaData[6] =   uint8_t(VBAT*50.0); // maximum should be 4.2 * 50 = 210
  

      if (LoRaWAN.connected())
     {
        LoRaWAN.beginPacket(3);
        LoRaWAN.write(LoRaData, sizeof(LoRaData));
        LoRaWAN.endPacket();
     }

     count = 0;
    
    }
    
    } // end of alarm section


//    SPIFlash.powerDown();  // Put SPI flash into power down mode
//    SPI.end();             // End SPI peripheral to save power in STOP mode
    digitalWrite(myLed, LOW); delay(100); digitalWrite(myLed, HIGH); delay(900);
    //    STM32L0.stop();        // Enter STOP mode and wait for an interrupt
//    SPI.begin();           // When exiting STOP mode, re-enable the SPI peripheral
   
}

/* Useful functions */

 void alarmMatch()
{
  alarmFlag = true; // Just set flag when interrupt received, don't try reading data in an interrupt handler
}

void myinthandler1()
{
  newBMA280Data = true;  
}

void myinthandler2()
{
  newBMA280Tap = true;
}


