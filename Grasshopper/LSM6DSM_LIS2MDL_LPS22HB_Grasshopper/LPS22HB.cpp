/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LPS22HB is a low power barometerr.

  Library may be used freely and without limit with attribution.

*/

#include "LPS22HB.h"
#include "Wire.h"

LPS22H::LPS22H(uint8_t intPin)
{
  pinMode(intPin, INPUT);
  _intPin = intPin; 
}

uint8_t LPS22H::getChipID()
{
  // Read the WHO_AM_I register of the altimeter this is a good test of communication
  uint8_t temp = readByte(LPS22H_ADDRESS, LPS22H_WHOAMI);  // Read WHO_AM_I register for LPS22H
  return temp;
}

uint8_t LPS22H::status()
{
  // Read the status register of the altimeter  
  uint8_t temp = readByte(LPS22H_ADDRESS, LPS22H_STATUS);   
  return temp;
}

int32_t LPS22H::readAltimeterPressure()
{
    uint8_t rawData[3];  // 24-bit pressure register data stored here
    readBytes(LPS22H_ADDRESS, (LPS22H_PRESS_OUT_XL | 0x80), 3, &rawData[0]); // bit 7 must be one to read multiple bytes
    return (int32_t) ((int32_t) rawData[2] << 16 | (int32_t) rawData[1] << 8 | rawData[0]);
}

int16_t LPS22H::readAltimeterTemperature()
{
    uint8_t rawData[2];  // 16-bit pressure register data stored here
    readBytes(LPS22H_ADDRESS, (LPS22H_TEMP_OUT_L | 0x80), 2, &rawData[0]); // bit 7 must be one to read multiple bytes
    return (int16_t)((int16_t) rawData[1] << 8 | rawData[0]);
}


void LPS22H::Init(uint8_t PODR)
{
  // set sample rate by setting bits 6:4 
  // enable low-pass filter by setting bit 3 to one
  // bit 2 == 0 means bandwidth is odr/9, bit 2 == 1 means bandwidth is odr/20
  // make sure data not updated during read by setting block data udate (bit 1) to 1
    writeByte(LPS22H_ADDRESS, LPS22H_CTRL_REG1, PODR << 4 | 0x08 | 0x02);  
    writeByte(LPS22H_ADDRESS, LPS22H_CTRL_REG3, 0x04);  // enable data ready as interrupt source
}

// I2C scan function
void LPS22H::I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
      
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}

  void LPS22H::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
  {
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
  }

  uint8_t LPS22H::readByte(uint8_t address, uint8_t subAddress)
  {
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (size_t) 1);  // Read one uint8_t from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
  }

  void LPS22H::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
  {  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, (size_t)count);  // Read bytes from slave register address 
  Wire.read(dest, count);            // Put read results in the Rx buffer
  }
