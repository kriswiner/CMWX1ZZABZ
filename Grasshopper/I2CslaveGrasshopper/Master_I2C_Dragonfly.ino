#include <Wire.h>

uint8_t intPin = 9;

uint8_t rx_data[4] = {0, 0, 0, 0};
bool newData = false;
float VDDA = 0.0f, Temp = 0.0f;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);

  pinMode(intPin, INPUT);
 
  Wire.begin(TWI_PINS_20_21); // set master mode 
  Wire.setClock(400000);      // I2C frequency at 400 kHz 
  delay(1000);
  
  I2Cscan();

  Serial.println("I2C scan done");

  attachInterrupt(intPin, myInterruptHandler, RISING);

}


void loop() {
  
  if(newData)
  {
  newData = false;

  readBytes(0x7C, 4, rx_data);
  for (uint8_t i = 0; i < 4; i++)
  {
    Serial.print("0x"); Serial.println(rx_data[i], HEX);
  }
  Serial.println(" ");
  
  VDDA = (float) ( (uint16_t)rx_data[0] << 8 | (uint16_t)rx_data[1] );
  VDDA /= 100.0f;
  Temp = (float) ( (uint16_t)rx_data[2] << 8 | (uint16_t)rx_data[3]);
  Temp /= 100.0f;
  Serial.print(" VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print(" Temp = "); Serial.print(Temp, 2); Serial.println(" C");
  Serial.println(" ");
  }

}

// Useful function

void myInterruptHandler()
{
newData = true;
}

// I2C scan function
void I2Cscan()
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
//    Wire.beginTransmission(address);
//    error = Wire.endTransmission();
      error = Wire.transfer(address, NULL, 0, NULL, 0);
      
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

  void readBytes(uint8_t address, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
//  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (Wire.available()) 
        {
        dest[i++] = Wire.read(); 
        }         // Put read results in the Rx buffer
}
