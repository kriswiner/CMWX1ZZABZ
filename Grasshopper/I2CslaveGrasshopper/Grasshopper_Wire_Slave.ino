/* Wire Slave example (see Wire_Master.ino for the master)
 *    
 * The myReceiveCallback tracking the received data sets
 * tx_index into tx_data[], if only one byte had been
 * transferred. The myRequestCallback puts 32 bytes from
 * tx_data[] starting at tx_index into the transmit buffer.
 * Finally the myTransmitCallback adjusts tx_index with
 * the number of transferred bytes.
 *
 * The code roughly simluates a slave device with a FIFO.
 * Sunce the myRequestCallback cannot know how many bytes
 * need to be send, it fills up the buffer to the max. 
 * Only at the myTransmitCallback the number  of bytes
 * transmitted is known.
 *
 *    
 * This example code is in the public domain.
 */

#include "Wire.h"
#include <STM32L0.h>
#include "TimerMillis.h"

TimerMillis SensorTimer;

int tx_index = 0;
bool newData = true;
uint8_t intPin =  10, myLed = 13;

// MCU VDDA and temperature definitions
float VDDA, STM32L0Temp;

//uint8_t tx_data[] = "The quick brown fox jumps over the lazy dog\r\n";
uint8_t tx_data[4];

void myReceiveCallback(int count)
{
    if (count == 1)
    {
        tx_index = Wire.read();
        
        while (tx_index >= sizeof(tx_data))
        {
            tx_index -= sizeof(tx_data);
        }
    }
}

void myRequestCallback(void)
{
    for (int i = 0, n = tx_index; i < BUFFER_LENGTH; i++)
    {
        Wire.write(tx_data[n]);
        
        n++;
        
        if (n >= sizeof(tx_data)) { n = 0; }
    }
}

void myTransmitCallback(int count)
{
    tx_index += count;

    while (tx_index >= sizeof(tx_data))
    {
        tx_index -= sizeof(tx_data);
    }
}

void mySensorCallback()
{
  newData = true; 
  STM32L0.wakeup();
}

void setup()
{
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, LOW); // start with led off since active HIGH
    pinMode(intPin, OUTPUT);
  
    Wire.begin(0x7c);

    Wire.onReceive(myReceiveCallback);
    Wire.onRequest(myRequestCallback);
    Wire.onTransmit(myTransmitCallback);

    SensorTimer.start(mySensorCallback, 1000, 1000);      //  1 second period, delayed 1 seconds
}

void loop()
{
  if(newData) 
  {
  newData = false;
  
  VDDA = STM32L0.getVDDA();
  STM32L0Temp = STM32L0.getTemperature();
  
  tx_data[0] = (((uint16_t) (100.0f * VDDA)) & 0xFF00) >> 8;
  tx_data[1] = (((uint16_t) (100.0f * VDDA)) & 0x00FF);
  tx_data[2] = (((uint16_t) (100.0f * STM32L0Temp)) & 0xFF00) >> 8;
  tx_data[3] = (((uint16_t) (100.0f * STM32L0Temp)) & 0x00FF);

  digitalWrite(intPin, HIGH); delay(1); digitalWrite(intPin,LOW);
  digitalWrite(myLed, HIGH);  delay(1); digitalWrite(myLed,LOW);
  }

  STM32L0.stop();
}
