#ifndef _RADIOLIB_EX_LORAWAN_CONFIG_H
#define _RADIOLIB_EX_LORAWAN_CONFIG_H

#include <Arduino.h>
#include <boards.h>
#include <RadioLib.h>
#include "utilsLorawan.h"


Radio radio = new RadioModule();

// how often to send an uplink - consider legal & FUP constraints - see notes
#ifdef DO_WORK_INTERVAL_SECONDS
const uint32_t uplinkIntervalSeconds = DO_WORK_INTERVAL_SECONDS;    // minutes x seconds
#else
const uint32_t uplinkIntervalSeconds = 30   ;    // minutes x seconds
#endif

// device address - either a development address or one assigned
// to the LoRaWAN Service Provider - TTN will generate one for you
#ifndef RADIOLIB_LORAWAN_DEV_ADDR   // Replace with your DevAddr
#define RADIOLIB_LORAWAN_DEV_ADDR  0x00000000
#endif

#ifndef RADIOLIB_LORAWAN_NWKSENC_KEY   // Replace with your NwkSEnc Key 
#define RADIOLIB_LORAWAN_NWKSENC_KEY 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
#endif
#ifndef RADIOLIB_LORAWAN_APPS_KEY   // Replace with your AppS Key 
#define RADIOLIB_LORAWAN_APPS_KEY   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
#endif

// regional choices: EU868, US915, AU915, AS923, AS923_2, AS923_3, AS923_4, IN865, KR920, CN500
const LoRaWANBand_t Region = AU915;
const uint8_t subBand = 2;  // For US915 and AU915, change this to 2, otherwise leave on 0

// copy over the keys in to the something that will not compile if incorrectly formatted
uint32_t devAddr =        RADIOLIB_LORAWAN_DEV_ADDR;
uint8_t nwkSEncKey[] =  { RADIOLIB_LORAWAN_NWKSENC_KEY };
uint8_t appSKey[] =     { RADIOLIB_LORAWAN_APPS_KEY };

// create the LoRaWAN node
LoRaWANNode node(&radio, &Region, subBand);

#endif