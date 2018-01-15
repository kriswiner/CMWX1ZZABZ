#include "LSM6DSM.h"
#include "LIS2MDL.h"
#include "LPS22HB.h"
#include <STM32L0.h>
#include <RTC.h>

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 13

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
uint32_t delt_t = 0;                      // used to control display output rate
uint32_t sumCount = 0;                    // used to control display output rate
float pitch, yaw, roll;                   // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


//LSM6DSM definitions
#define LSM6DSM_intPin1 4  // interrupt1 pin definitions, significant motion
#define LSM6DSM_intPin2 5   // interrupt2 pin definitions, data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_245DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_12_5Hz, AODR_26Hz, AODR_52Hz, AODR_104Hz, AODR_208Hz, AODR_416Hz, AODR_833Hz, AODR_1660Hz, AODR_3330Hz, AODR_6660Hz
      GODR_12_5Hz, GODR_26Hz, GODR_52Hz, GODR_104Hz, GODR_208Hz, GODR_416Hz, GODR_833Hz, GODR_1660Hz, GODR_3330Hz, GODR_6660Hz
*/ 
uint8_t Ascale = AFS_2G, Gscale = GFS_245DPS, AODR = AODR_208Hz, GODR = GODR_208Hz;

float aRes, gRes;              // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {-0.01308, -0.00493, 0.03083}, gyroBias[3] = {0.71, -2.69, 0.78}; // offset biases for the accel and gyro
int16_t LSM6DSMData[7];        // Stores the 16-bit signed sensor output
float   Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;  // variables to hold latest accel/gyro data values 

bool newLSM6DSMData = false;
bool newLSM6DSMTap  = false;

LSM6DSM LSM6DSM(LSM6DSM_intPin1, LSM6DSM_intPin2); // instantiate LSM6DSM class


//LIS2MDL definitions
#define LIS2MDL_intPin  9 // interrupt for magnetometer data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
*/ 
uint8_t MODR = MODR_100Hz;

float mRes = 0.0015f;            // mag sensitivity
float magBias[3] = {0.814, -0.093, -0.579}, magScale[3]  = {1.11,1.02,0.90}; // Bias corrections for magnetometer
int16_t LIS2MDLData[4];          // Stores the 16-bit signed sensor output
float Mtemperature;              // Stores the real internal chip temperature in degrees Celsius
float mx, my, mz;                // variables to hold latest mag data values 
uint8_t LIS2MDLstatus;

bool newLIS2MDLData = false;

LIS2MDL LIS2MDL(LIS2MDL_intPin); // instantiate LIS2MDL class


// LPS22H definitions
uint8_t LPS22H_intPin = 8;

/* Specify sensor parameters (sample rate is twice the bandwidth) 
   Choices are P_1Hz, P_10Hz P_25 Hz, P_50Hz, and P_75Hz
 */
uint8_t PODR = P_25Hz;     // set pressure amd temperature output data rate
uint8_t LPS22Hstatus;
float Temperature, Pressure, altitude;

bool newLPS22HData = false;

LPS22H LPS22H(LPS22H_intPin);


// RTC set time using STM32L4 natve RTC class
/* Change these values to set the current initial time */
uint8_t seconds = 0;
uint8_t minutes = 15;
uint8_t hours = 11;

/* Change these values to set the current initial date */
uint8_t day = 15;
uint8_t month = 1;
uint8_t year = 18;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = false; // for RTC alarm interrupt



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // start with led off
  
  Wire.begin(); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);
 
  LSM6DSM.I2Cscan();

  // Read the LSM6DSM Chip ID register, this is a good test of communication
  Serial.println("LSM6DSM accel/gyro...");
  byte c = LSM6DSM.getChipID();  // Read CHIP_ID register for LSM6DSM
  Serial.print("LSM6DSM "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x6A, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the LIS2MDL Chip ID register, this is a good test of communication
  Serial.println("LIS2MDL mag...");
  byte d = LIS2MDL.getChipID();  // Read CHIP_ID register for LSM6DSM
  Serial.print("LIS2MDL "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
  Serial.println(" ");
  delay(1000); 

  Serial.println("LPS22HB barometer...");
  uint8_t e = LPS22H.getChipID();
  Serial.print("LPS25H "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0xB1, HEX);
  delay(1000); 
  

  if(c == 0x6A && d == 0x40 && e == 0xB1) // check if all I2C sensors have acknowledged
  {
   Serial.println("LSM6DSM and LIS2MDL and LPS22HB are online..."); Serial.println(" ");
   
   digitalWrite(myLed, LOW);

   LSM6DSM.reset();  // software reset LSM6DSM to default registers

   // get sensor resolutions, only need to do this once
   aRes = LSM6DSM.getAres(Ascale);
   gRes = LSM6DSM.getGres(Gscale);

   LSM6DSM.init(Ascale, Gscale, AODR, GODR);

   LSM6DSM.selfTest();

//   LSM6DSM.offsetBias(gyroBias, accelBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
   delay(1000); 

   LIS2MDL.reset(); // software reset LIS2MDL to default registers

   mRes = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss); 
   
   LIS2MDL.init(MODR);

   LIS2MDL.selfTest();

   LIS2MDL.offsetBias(magBias, magScale);
   Serial.println("mag biases (mG)"); Serial.println(1000.0f * magBias[0]); Serial.println(1000.0f * magBias[1]); Serial.println(1000.0f * magBias[2]); 
   Serial.println("mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
   delay(2000); // add delay to see results before serial spew of data

   LPS22H.Init(PODR);  // Initialize LPS22H altimeter
   delay(1000);

   digitalWrite(myLed, HIGH);
   
  }
  else 
  {
  if(c != 0x6A) Serial.println(" LSM6DSM not functioning!");
  if(d != 0x40) Serial.println(" LIS2MDL not functioning!");    
  if(e != 0xB1) Serial.println(" LPS22HB not functioning!");   

  while(1){};
  }

 // Set the time
  SetDefaultRTC();
  
  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(LSM6DSM_intPin2, myinthandler1, RISING);  // define interrupt for intPin2 output of LSM6DSM
  attachInterrupt(LIS2MDL_intPin , myinthandler2, RISING);  // define interrupt for intPin  output of LIS2MDL
  attachInterrupt(LPS22H_intPin  , myinthandler3, RISING);  // define interrupt for intPin  output of LPS22HB

  LIS2MDL.readData(LIS2MDLData);  // read data register to clear interrupt before main loop
}

/* End of setup */

void loop() {

   // If intPin goes high, either all data registers have new data
   if(newLSM6DSMData == true) {   // On interrupt, read data
      newLSM6DSMData = false;     // reset newData flag

     LSM6DSM.readData(LSM6DSMData); // INT2 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     ax = (float)LSM6DSMData[4]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)LSM6DSMData[5]*aRes - accelBias[1];   
     az = (float)LSM6DSMData[6]*aRes - accelBias[2];  

   // Calculate the gyro value into actual degrees per second
     gx = (float)LSM6DSMData[1]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     gy = (float)LSM6DSMData[2]*gRes - gyroBias[1];  
     gz = (float)LSM6DSMData[3]*gRes - gyroBias[2]; 

    for(uint8_t i = 0; i < 20; i++) { // iterate a fixed number of times per data read cycle
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;

    MadgwickQuaternionUpdate(-ax, ay, az, gx*pi/180.0f, -gy*pi/180.0f, -gz*pi/180.0f,  mx,  my, -mz);
    }
    
   }

    // If intPin goes high, either all data registers have new data
   if(newLIS2MDLData == true) {   // On interrupt, read data
      newLIS2MDLData = false;     // reset newData flag

     LIS2MDLstatus = LIS2MDL.status();
     
     if(LIS2MDLstatus & 0x08) // if all axes have new data ready
     {
      LIS2MDL.readData(LIS2MDLData);  
   
   // Now we'll calculate the accleration value into actual G's
     mx = (float)LIS2MDLData[0]*mRes - magBias[0];  // get actual G value 
     my = (float)LIS2MDLData[1]*mRes - magBias[1];   
     mz = (float)LIS2MDLData[2]*mRes - magBias[2]; 
     mx *= magScale[0];
     my *= magScale[1];
     mz *= magScale[2];  
     }
   }

    
   // end sensor interrupt handling

    if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved and the MPU9250 is awake
       alarmFlag = false;

    // Read RTC
   if(SerialDebug)
    {
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
    }
    
    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print((int)1000*mx);  
    Serial.print(" my = "); Serial.print((int)1000*my); 
    Serial.print(" mz = "); Serial.print((int)1000*mz); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
    }

    // get pressure and temperature from the LPS22HB
    LPS22Hstatus = LPS22H.status();

    if(LPS22Hstatus & 0x01) { // if new pressure data available
    Pressure = (float) LPS22H.readAltimeterPressure()/4096.0f;
    Temperature = (float) LPS22H.readAltimeterTemperature()/100.0f; 
    
    altitude = 145366.45f*(1.0f - pow((Pressure/1013.25f), 0.190284f)); 

      if(SerialDebug) {
      Serial.print("Altimeter temperature = "); Serial.print( Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius  
      Serial.print("Altimeter temperature = "); Serial.print(9.0f*Temperature/5.0f + 32.0f, 2); Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); Serial.print(Pressure, 2);  Serial.println(" mbar");// pressure in millibar
      Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
      }
    }

    Gtemperature = ((float) LSM6DSMData[0]) / 256.0f + 25.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Gyro temperature is ");  Serial.print(Gtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    LIS2MDLData[3] = LIS2MDL.readTemperature();
    Mtemperature = ((float) LIS2MDLData[3]) / 8.0f + 25.0f; // Mag chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Mag temperature is ");  Serial.print(Mtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);
    pitch *= 180.0f / pi;
    yaw   *= 180.0f / pi; 
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / pi;
    lin_ax = ax + a31;
    lin_ay = ay + a32;
    lin_az = az - a33;

    if(SerialDebug) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    Serial.print("Grav_x, Grav_y, Grav_z: ");
    Serial.print(-a31*1000.0f, 2);
    Serial.print(", ");
    Serial.print(-a32*1000.0f, 2);
    Serial.print(", ");
    Serial.print(a33*1000.0f, 2);  Serial.println(" mg");
    Serial.print("Lin_ax, Lin_ay, Lin_az: ");
    Serial.print(lin_ax*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_ay*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_az*1000.0f, 2);  Serial.println(" mg");
    
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    }

     Serial.print(millis()/1000);Serial.print(",");
     Serial.print(yaw, 2); Serial.print(","); Serial.print(pitch, 2); Serial.print(","); Serial.print(roll, 2); Serial.print(","); Serial.println(Pressure, 2);

    sumCount = 0;
    sum = 0;      

    digitalWrite(myLed, HIGH); delay(10); digitalWrite(myLed, LOW);   
    }
}

/*  End of main loop */


void myinthandler1()
{
  newLSM6DSMData = true;
}

void myinthandler2()
{
  newLIS2MDLData = true;
}

void myinthandler3()
{
  newLPS22HData = true;
}

void alarmMatch()
{
  alarmFlag = true;
}

void SetDefaultRTC()  // Sets the RTC to the FW build date-time...
{
  char Build_mo[3];

  // Convert month string to integer

  Build_mo[0] = build_date[0];
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];

  String build_mo = Build_mo;

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
    month = 1;     // Default to January if something goes wrong...
  }

  // Convert ASCII strings to integers
  day     = (build_date[4] - 48)*10 + build_date[5] - 48;  // ASCII "0" = 48
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1] - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4] - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7] - 48;

  // Set the date/time

  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
  
