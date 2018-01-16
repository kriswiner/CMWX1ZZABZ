/* Asset tracker example, for Cricket
   Uses the GNSS add-on with CAM M8Q described here:

   https://hackaday.io/project/25790-asset-tracker

   Idea is ultra-low power for longest LiPo battery life so I would run this with
   4.2 MHz clock speed; this reduction plus use of STM32 stop mode means no serial
   through the USB. That's why there is a low power Sharp TFT display here.

    This example code is in the public domain.
*/
#include <Arduino.h>
#include <Wire.h>
#include <STM32L0.h>
#include "GNSS.h"
#include <RTC.h>
#include <SPI.h>
#include "SPIFlash.h"
//#include <Adafruit_GFX.h>
//#include <Adafruit_SharpMem.h>
/*
// define pins for Sharp LCD display, any pins can be used
uint8_t DSCK  = 12;
uint8_t DMOSI = 11;
uint8_t DSS   = 9;

Adafruit_SharpMem display(DSCK, DMOSI, DSS);

#define BLACK 0
#define WHITE 1
*/
// Grasshopper
#define myLed 10 // blue led 
#define pps 4
#define CAMM8Qen 5

bool SerialDebug = true;

uint16_t Hour = 0, Minute = 0, Second = 0, Millisec, Year = 0, Month = 0, Day = 0, Alt = 0;
uint16_t hour = 0, minute = 0, second = 0, year = 0, month = 0, day = 0, millisec;
bool ppsFlag = false, firstSync = false, alarmFlag = false;
uint8_t count = 0, fixType = 0, fixQuality, latBytes[4], longBytes[4], tempBytes[4], pressBytes[4];
uint8_t yawBytes[2], pitchBytes[2], rollBytes[2];
int32_t latOut;

float Temperature, Long, Lat;

// battery voltage monitor definitions
uint16_t rawVbat;
float VDDA, VBAT;
#define VbatMon A1

// 64 MBit (8 MByte) SPI Flash 32,768, 256-byte pages
#define csPin 21 // SPI Flash chip select pin

uint16_t page_number = 0;     // set the page number for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write

SPIFlash SPIFlash(csPin);


void setup()
{
  Serial.begin(38400);
  delay(2000);
  Serial.println("Serial enabled!");

  Wire.begin(); // set master mode on pins 14/15, I2C frequency at 400 kHz
  Wire.setClock(400000);

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with blue led off, since active LOW

  pinMode(CAMM8Qen, OUTPUT);
  digitalWrite(CAMM8Qen, HIGH);

  // Voltage divider 27K/100K to monitor LiPo battery voltage
  pinMode(VbatMon, INPUT);
  analogReadResolution(12); // take advantage of 12-bit ADCs
/*
  // Set up for data display
  display.begin(); // Initialize the display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer

  // Start device display with ID of sensor
  display.setCursor(0, 10); display.print("Butterfly");
  display.setCursor(0, 20); display.print("CAM M8Q");
  display.setCursor(0, 40); display.print("Concurrent");
  display.setCursor(0, 60); display.print("GNSS");
  display.setCursor(0, 80); display.print("EM7180 + MPU9250");
  display.setCursor(0, 100); display.print("Abs.Orientation");
  display.refresh();
  delay(1000);
*/
  pinMode(pps, INPUT);

  GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ); // Start GNSS

  // check SPI Flash ID
  SPIFlash.SPIFlashinit();
  SPIFlash.getChipID();

  // Set the RTC time
  RTC.setHours(hour);
  RTC.setMinutes(minute);
  RTC.setSeconds(second);
  RTC.setMinutes(minute);

  // Set the RTC date
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
/*
  int16_t calib = 0;
  RTC.setCalibration(calib);  // clock slow, add pulses, clock fast, subtract pulses

  // Check calibration
  int16_t calreg = RTC.getCalibration();
  Serial.print("Calibration pulses = "); Serial.println(calreg);
*/
  // set alarm to update the RTC every second
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

  RTC.attachInterrupt(alarmMatch);

  attachInterrupt(pps, CAMM8QintHandler, RISING);

}

void loop()
{
    /*GNSS*/
//    if (GNSS.available()) // check if new GNSS data is available
    if (ppsFlag == true)
    {
     ppsFlag = false;
     delay(50); // delay a bit to allow GNSS data to become available   {

     GNSSLocation myLocation = GNSS.read(); // read available GNSS data

    if (myLocation) // if there is a fix
    {
      Lat = myLocation.latitude();
      int32_t_float_to_bytes(Lat, &latBytes[0]);
      Long = myLocation.longitude();
      int32_t_float_to_bytes(Long, &longBytes[0]);
      Alt = myLocation.altitude();
      Serial.print("latitude = ");
      Serial.print(Lat, 7);
      Serial.print(", longitude = ");
      Serial.print(Long, 7);
      Serial.print(", altitude = ");
      Serial.print(Alt, 1);
      Serial.print(", satellites = ");
      Serial.print(myLocation.satellites());
      Serial.print(", pdop = ");
      Serial.print(myLocation.pdop());
      Serial.print(", fixType = ");
      fixType = myLocation.fixType();
      if (fixType == 0) Serial.print("none");
      if (fixType == 1) Serial.print("time");
      if (fixType == 2) Serial.print("2D");
      if (fixType == 3) Serial.print("3D");
      Serial.print(", fixQuality = ");
      fixQuality = myLocation.fixQuality();
      if (fixQuality == 0) Serial.print("none");
      if (fixQuality == 1) Serial.print("auto");
      if (fixQuality == 2) Serial.print("diff");
      if (fixQuality == 3) Serial.print("prec");
      if (fixQuality == 4) Serial.print("rtk_fixed");
      if (fixQuality == 5) Serial.print("rtk_float");
      if (fixQuality == 6) Serial.print("est");
      if (fixQuality == 7) Serial.print("man");
      if (fixQuality == 8) Serial.print("sim");
      Serial.println();

      Hour   = myLocation.hour();
      Minute = myLocation.minute();
      Second = myLocation.second();
      Millisec = myLocation.millis();
      Serial.print("GNSS Time = ");
      if (Hour < 10)   {
        Serial.print("0");
        Serial.print(Hour);
      } else Serial.print(Hour);
      Serial.print(":");
      if (Minute < 10) {
        Serial.print("0");
        Serial.print(Minute);
      } else Serial.print(Minute);
      Serial.print(":");
      if (Second < 10) {
        Serial.print("0");
        Serial.print(Second);
      } else Serial.print(Second);
      Serial.print(":");
      if (Millisec < 10) {
        Serial.print("0");
        Serial.println(Millisec);
      } else Serial.println(Millisec);

      Year = myLocation.year();
      Month = myLocation.month();
      Day = myLocation.day();
      Serial.print("GNSS Date = ");
      Serial.print(Year); Serial.print(":"); Serial.print(Month); Serial.print(":"); Serial.println(Day);
      Serial.println();

      // Test if the RTC has been synced after GNSS time available
      if (firstSync == false)
      {
        firstSync = true;
        syncRTC();  // just need to sync once
      }

      // Send some data to the SPI flash
      if (sector_number < 8 && page_number < 0x7FFF) { // 32,7686 256-byte pages in a 8 MByte flash
        flashPage[sector_number * 32 + 0]  = latBytes[0];  // latitude in bytes
        flashPage[sector_number * 32 + 1]  = latBytes[1];
        flashPage[sector_number * 32 + 2]  = latBytes[2];
        flashPage[sector_number * 32 + 3]  = latBytes[3];
        flashPage[sector_number * 32 + 4]  = longBytes[0]; // longitude in bytes
        flashPage[sector_number * 32 + 5]  = longBytes[1];
        flashPage[sector_number * 32 + 6]  = longBytes[2];
        flashPage[sector_number * 32 + 7]  = longBytes[3];
        flashPage[sector_number * 32 + 8]  = yawBytes[0];   // heading
        flashPage[sector_number * 32 + 9]  = yawBytes[1];
        flashPage[sector_number * 32 + 10] = pitchBytes[0]; // pitch
        flashPage[sector_number * 32 + 11] = pitchBytes[1];
        flashPage[sector_number * 32 + 12] = rollBytes[0]; // roll
        flashPage[sector_number * 32 + 13] = rollBytes[1];
        flashPage[sector_number * 32 + 14] = pressBytes[0];
        flashPage[sector_number * 32 + 15] = pressBytes[1];
        flashPage[sector_number * 32 + 16] = pressBytes[2];
        flashPage[sector_number * 32 + 17] = pressBytes[3];
        flashPage[sector_number * 32 + 18] = tempBytes[0];
        flashPage[sector_number * 32 + 19] = tempBytes[1];
        flashPage[sector_number * 32 + 20] = tempBytes[2];
        flashPage[sector_number * 32 + 21] = tempBytes[3];
        flashPage[sector_number * 32 + 22] = Second;
        flashPage[sector_number * 32 + 23] = Minute;
        flashPage[sector_number * 32 + 24] = Hour;
        flashPage[sector_number * 32 + 25] = Day;
        flashPage[sector_number * 32 + 26] = Month;
        flashPage[sector_number * 32 + 27] = (uint8_t) (Year - 2000);
        flashPage[sector_number * 32 + 28] = (Alt & 0xFF00) >> 8; // MSB GPS altitude
        flashPage[sector_number * 32 + 29] =  Alt & 0x00FF;       // LSB GPS altitude
        flashPage[sector_number * 32 + 30] = (rawVbat & 0xFF00) >> 8; // battery voltage
        flashPage[sector_number * 32 + 31] =  rawVbat & 0x00FF;
        sector_number++;
      }
      else if (sector_number == 8 && page_number < 0xFFFF)
      {
        SPIFlash.flash_page_program(flashPage, page_number);
        Serial.print("Wrote flash page: "); Serial.println(page_number);
        sector_number = 0;
        page_number++;
      }
      else
      {
        Serial.println("Reached last page of SPI flash!"); Serial.println("Data logging stopped!");
      }
    }
  }

  /*RTC*/
  if (alarmFlag) { // update RTC output whenever there is a GNSS pulse
    alarmFlag = false;

    VDDA = STM32L0.getVREF();
    Temperature = STM32L0.getTemperature();

    Serial.print("VDDA = "); Serial.println(VDDA, 2);
    Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);

    rawVbat = analogRead(VbatMon);
    VBAT = (127.0f / 100.0f) * 3.30f * ((float)rawVbat) / 4095.0f;
    Serial.print("VBAT = "); Serial.println(VBAT, 2);

    hour   = RTC.getHours();
    minute = RTC.getMinutes();
    second = RTC.getSeconds();
    Serial.print("RTC Time = ");
    if (hour < 10)   {
      Serial.print("0");
      Serial.print(hour);
    } else Serial.print(hour);
    Serial.print(":");
    if (minute < 10) {
      Serial.print("0");
      Serial.print(minute);
    } else Serial.print(minute);
    Serial.print(":");
    if (second < 10) {
      Serial.print("0");
      Serial.print(second);
    } else Serial.print(second);
    Serial.println();

    year = RTC.getYear();
    month = RTC.getMonth();
    day = RTC.getDay();
    Serial.print("RTC Date = ");
    Serial.print(year); Serial.print(":"); Serial.print(month); Serial.print(":"); Serial.println(day);
    Serial.println();

/*
    // set up TFT display data
    display.clearDisplay();
    display.setCursor(0, 10); display.print("lat "); display.print(Lat, 5);
    display.setCursor(0, 20); display.print("lon "); display.print(Long, 5);
    display.setCursor(0, 30); display.print("alt "); display.print(Alt, 1); display.print(" m");
    display.print(" fix "); display.print(fixType);
    display.setCursor(0, 40); display.print("GPS ");
    if (Hour < 10)   {
      display.print("0");
      display.print(Hour);
    } else display.print(Hour);
    display.print(":");
    if (Minute < 10) {
      display.print("0");
      display.print(Minute);
    } else display.print(Minute);
    display.print(":");
    if (Second < 10) {
      display.print("0");
      display.print(Second);
    } else display.print(Second);
    display.setCursor(0, 50);
    display.print(Year); display.print(":"); display.print(Month); display.print(":"); display.print(Day);
    display.print(" q "); display.print(fixQuality);
    display.setCursor(0, 60); display.print("RTC ");
    if (hour < 10)   {
      display.print("0");
      display.print(hour);
    } else display.print(hour);
    display.print(":");
    if (minute < 10) {
      display.print("0");
      display.print(minute);
    } else display.print(minute);
    display.print(":");
    if (second < 10) {
      display.print("0");
      display.print(second);
    } else display.print(second);

    display.setCursor(0, 70);
    display.print("Y"); display.print(Yaw, 1);
    display.print(" P"); display.print(Pitch, 1);
    display.print(" R"); display.print(Roll, 1);

    display.setCursor(0, 80);
    display.print("P "); display.print(pressure, 1); display.print(" mb ");
    display.print(" T "); display.print(temperature, 1); display.print(" C ");
    display.setCursor(0, 90);
    display.print("alt "); display.print(altitude, 1); display.print(" ft");

    display.setCursor(0, 100);
    display.print("VDDA "); display.print(VDDA, 2); display.print(" V");
    display.print(" T "); display.print(Temperature, 1); display.print(" C");

    display.setCursor(0, 110);
    display.print("VBAT "); display.print(VBAT, 2); display.print(" V");

    display.refresh();
    */
    digitalWrite(myLed, LOW); delay(10); digitalWrite(myLed, HIGH);
  }

//  STM32L0.sleep();  // sleep until next interrupt
}
/* end of loop*/

/* Useful functions */
void CAMM8QintHandler()
{
  ppsFlag = true;
}

void alarmMatch()
{
  alarmFlag = true;
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

void int32_t_float_to_bytes(float temp, uint8_t * dest)
{
  int32_t tempOut = temp * 10000000;
  dest[0] = (tempOut & 0xFF000000) >> 24;
  dest[1] = (tempOut & 0x00FF0000) >> 16;
  dest[2] = (tempOut & 0x0000FF00) >> 8;
  dest[3] = (tempOut & 0x000000FF);
}

void int16_t_float_to_bytes(float temp, uint8_t * dest)
{
  int32_t tempOut = temp * 50;
  dest[0] = (tempOut & 0xFF00) >> 8;
  dest[1] = (tempOut & 0x00FF);
}

