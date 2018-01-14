/* LED Blink, for LoRa Sensor Tile
 
   This example code is in the public domain.
*/
#include <Arduino.h>
#include <STM32L0.h>
#include "BME280.h"

#define myLed 10 // blue led
#define myBat A1 // 270K/1000K voltage divider on A1 ADC

#define SerialDebug true

float VBAT, VDDA, Temperature;
uint32_t UID[3] = {0, 0, 0}; 
uint16_t rawVbat = 0;

// BME280 definitions
/* Specify BME280 configuration
 *  Choices are:
 P_OSR_01, P_OSR_02, P_OSR_04, P_OSR_08, P_OSR_16 // pressure oversampling
 H_OSR_01, H_OSR_02, H_OSR_04, H_OSR_08, H_OSR_16 // humidity oversampling
 T_OSR_01, T_OSR_02, T_OSR_04, T_OSR_08, T_OSR_16 // temperature oversampling
 full, BW0_223ODR,BW0_092ODR, BW0_042ODR, BW0_021ODR // bandwidth at 0.021 x sample rate
 BME280Sleep, forced,, forced2, normal //operation modes
 t_00_5ms = 0, t_62_5ms, t_125ms, t_250ms, t_500ms, t_1000ms, t_10ms, t_20ms // determines sample rate
 */
uint8_t Posr = P_OSR_01, Hosr = H_OSR_01, Tosr = T_OSR_01, Mode = Sleep, IIRFilter = full, SBy = t_1000ms;     // set pressure amd temperature output data rate

uint32_t rawPress, rawTemp, compHumidity, compTemp, compPress;   // pressure, humidity, and temperature raw count output for BME280
uint16_t rawHumidity;  // variables to hold raw BME280 humidity value
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

BME280 myBME280; // instantiate BME280 class

void setup() 
{
  Serial.begin(38400);
  delay(2000);
  Serial.println("Serial enabled!");
 
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with leds off, since active LOW

  pinMode(myBat, INPUT);
  analogReadResolution(12); // take advantage of 12-bit ADCs

  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX);Serial.println(UID[2], HEX); 

  Wire.begin(); // set master mode 
  Wire.setClock(100000); // I2C frequency at 400 kHz  
  delay(1000);

  myBME280.I2Cscan(); // should detect BME280 at 0x77 

  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte d = myBME280.getChipID();  // Read WHO_AM_I register for BME280
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x60, HEX);
  Serial.println(" ");
  delay(1000);  

  if(d == 0x60) // check if all I2C sensors have acknowledged
  {
    Serial.println("BME280 is online..."); Serial.println(" ");

    myBME280.resetBME280(); // reset BME280 before initilization
    delay(100);

    myBME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy); // Initialize BME280 altimeter
  }
  else 
  {
    if(d != 0x60) Serial.println(" BME280 not functioning!");    
  }

}

void loop() 
{

  // BME280 Data
  myBME280.BME280forced();  // get one data sample, then go back to sleep
    
  rawTemp =  myBME280.readBME280Temperature();
  compTemp = myBME280.BME280_compensate_T(rawTemp);
  temperature_C = (float) compTemp/100.0f;
  temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
  rawPress =  myBME280.readBME280Pressure();
  compPress = myBME280.BME280_compensate_P(rawPress);
  pressure = (float) compPress/25600.f; // Pressure in mbar
  altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
  rawHumidity =  myBME280.readBME280Humidity();
  compHumidity = myBME280.BME280_compensate_H(rawHumidity);
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

  rawVbat = analogRead(myBat);
  Serial.print("rawVbat = "); Serial.println(rawVbat); 
  VBAT = (1270.0f/1000.0f) * 3.30f * ((float)rawVbat)/4095.0f;
  Serial.print("VBAT = "); Serial.println(VBAT, 2); 
  
  VDDA = STM32L0.getVREF();
  Temperature = STM32L0.getTemperature();

  Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(Temperature, 2);

  digitalWrite(myLed, LOW); delay(100); digitalWrite(myLed, HIGH);

  delay(900);
//  STM32L0.stop(2000);
}


