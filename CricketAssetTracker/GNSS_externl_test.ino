#include "GNSS.h"
#include "TimerMillis.h"
#include<STM32L0.h>
#include <RTC.h>

GNSSLocation myLocation;
GNSSSatellites mySatellites;

TimerMillis GNSSTimerWakeup;

#define myLed 10

bool firstSync = false, alarmFlag = true;
uint16_t Hour = 0, Minute = 0, Second = 0, Millisec, Year = 0, Month = 0, Day = 0, Alt = 0;
uint16_t hour = 0, minute = 0, second = 0, year = 0, month = 0, day = 0, millisec;
uint32_t count = 0;

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

void setup( void )
{
    Serial.begin(9600);

    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    // Set the RTC time
    RTC.setHours(hour);
    RTC.setMinutes(minute);
    RTC.setSeconds(second);
    RTC.setMinutes(minute);

    // Set the RTC date
    RTC.setDay(day);
    RTC.setMonth(month);
    RTC.setYear(year);
  

    GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ);
    while (GNSS.busy()) { }

    GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GLONASS);
    while (GNSS.busy()) { }

    GNSS.setAntenna(GNSS.ANTENNA_EXTERNAL);

    // set alarm to update the RTC periodically
    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

    RTC.attachInterrupt(alarmMatch);

    GNSSTimerWakeup.start(callbackWakeup, 0, 180000); // every three minutes, no delay
}

void loop( void )
{
  if (GNSS.location(myLocation))
  {
	Serial.print("LOCATION: ");
	Serial.print(fixTypeString[myLocation.fixType()]);

	if (myLocation.fixType() != GNSSLocation::TYPE_NONE)
	{
      Hour   = myLocation.hours();
      Minute = myLocation.minutes();
      Second = myLocation.seconds();
      Year = myLocation.year();
      Month = myLocation.month();
      Day = myLocation.day();
      
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


      // Test if the RTC has been synced after GNSS time available
      if (firstSync == false)
      {
        firstSync = true;
        syncRTC();  // just need to sync once
      }


	    if (myLocation.fixType() != GNSSLocation::TYPE_TIME)
	    {
		Serial.print(" LLA=");
		Serial.print(myLocation.latitude(), 7);
		Serial.print(",");
		Serial.print(myLocation.longitude(), 7);
		Serial.print(",");
		Serial.print(myLocation.altitude(), 3);
		Serial.print(" EPE=");
		Serial.print(myLocation.ehpe(), 3);
		Serial.print(",");
		Serial.print(myLocation.evpe(), 3);
		Serial.print(" SATELLITES=");
		Serial.print(myLocation.satellites());
		Serial.print(" DOP=");
		Serial.print(myLocation.hdop(), 2);
		Serial.print(",");
		Serial.print(myLocation.vdop(), 2);

         if(myLocation.fixType() != GNSSLocation::TYPE_2D) {
          Serial.println("GNSS go to sleep!");
          GNSS.suspend(); // once we have a 3D location fix put CAM M8Q to sleep
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
		Serial.print(svid -64);
	    }
	    else
	    {
		Serial.print(" R");
		Serial.print(svid -64);
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
	    Serial.print(svid -172);
	}
	else if ((svid >= 193) && (svid <= 197))
	{
	    Serial.print("    ");
	    Serial.print("  Q");
	    Serial.print(svid -192);
	}
	else if ((svid >= 211) && (svid <= 246))
	{
	    Serial.print("    ");

	    if ((svid - 210) <= 9)
	    {
		Serial.print("  E");
		Serial.print(svid -210);
	    }
	    else
	    {
		Serial.print(" E");
		Serial.print(svid -210);
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
    
} /* end of GNSS Satellites handling

  /*RTC*/
  if (alarmFlag) { // update RTC output periodically
    alarmFlag = false;
    
    Serial.println("RTC:");
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
    Serial.println(" ");

    year = RTC.getYear();
    month = RTC.getMonth();
    day = RTC.getDay();
    Serial.print("RTC Date = ");
    Serial.print(year); Serial.print(":"); Serial.print(month); Serial.print(":"); Serial.println(day);
    Serial.println();

    digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH); 
    
  } /* end of RTC alarm handling */
/*
  float VDDA = STM32L0.getVDDA();
  float VBUS = STM32L0.getVBUS();
  float VBAT = STM32L0.getVBAT();
  float STM32L0Temp = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
  if(VBUS ==  1)  Serial.println("USB Connected!"); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
  Serial.println(" ");
    */

    STM32L0.stop();
}


/* Useful functions */

void callbackWakeup(void)
{
    GNSS.resume();
    Serial.println("wakeup!");
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
