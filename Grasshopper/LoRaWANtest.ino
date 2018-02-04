#include "LoRaWAN.h"

#define myLed 13 // red led

const char *appEui = "70B3D57ED00093FB";
const char *appKey = "6E47CC3C9B1F7F155600EE8A4FEEAFA3";
const char *devEui = "4473632393936313"; 

void setup( void )
{
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);
  
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
    LoRaWAN.setSubBand(2); // for TTN 

    LoRaWAN.joinOTAA(appEui, appKey, devEui);
}

void loop( void )
{
    delay(60000);

    if (!LoRaWAN.busy() && LoRaWAN.joined())
    {
        LoRaWAN.beginPacket(3);
        LoRaWAN.write(0xef);
        LoRaWAN.write(0xbe);
        LoRaWAN.write(0xad);
        LoRaWAN.write(0xde);
        LoRaWAN.endPacket();
    }

    digitalWrite(myLed, HIGH); delay(10); digitalWrite(myLed, LOW);
}
