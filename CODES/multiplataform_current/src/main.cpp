/*
  RadioLib LoRaWAN ABP Example

  ABP = Activation by Personalisation, an alternative
  to OTAA (Over the Air Activation). OTAA is preferable.

  This example will send uplink packets to a LoRaWAN network. 
  Before you start, you will have to register your device at 
  https://www.thethingsnetwork.org/
  After your device is registered, you can run this example.
  The device will join the network and start uploading data.

  LoRaWAN v1.0.4/v1.1 requires the use of persistent storage.
  As this example does not use persistent storage, running this 
  examples REQUIRES you to check "Resets frame counters"
  on your LoRaWAN dashboard. Refer to the notes or the 
  network's documentation on how to do this.
  To comply with LoRaWAN's persistent storage, refer to
  https://github.com/radiolib-org/radiolib-persistence

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/

  For LoRaWAN details, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/LoRaWAN

*/


#include "configABP.h"

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#define CANTMUESTRAS 1000 //amostras a serem coletadas (amostras mais precisas, mas com maior tempo de execução)
#define AMAXSENS 100       // A corrente máxima do sensor neste caso é o SCT013, que oferece 30 A máx. a 1000 mV.
#define MVMAXSENS 512    // MV máximo oferecido pelo sensor em sua corrente máxima suportada
#define VOLTRED 220       // Tensão da rede

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

float multiplier = 0.010162022;
float Peficaz;
float Int_calculada;

uint32_t counter = 0;

float med_Ieficaz();

void setup() { 
  SetupBoard();
  SerialInit();
  RadioBeginSPI();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.setGain(GAIN_TWO);         // 2x gain   +/- 2.048V  1 bit =     0.0625mV
  ads.begin();
  delay(5000);  // Give time to switch to the serial monitor
  serial.println("\nSetup ... ");
  serial.println("Initialise the radio");
  int state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);
  serial.println(F("Initialise LoRaWAN Network credentials"));
  node.setDutyCycle(false);
  node.setDwellTime(false);
  node.beginABP(devAddr, NULL, NULL, nwkSEncKey, appSKey);
  node.activateABP(DR_SF8);
  debug(state != RADIOLIB_ERR_NONE, F("Activate ABP failed"), state, true);
  serial.println(F("Ready!\n"));
  serial.printf("0x%x\n\r", node.getDevAddr());
}

void loop() { 
  serial.printf("\nSending uplink\n\r");

  if(counter < 1000) {
    node.setDatarate(DR_SF7);
    serial.printf("Set SF for SF7\n\r");
  } else if ( counter < 2000) {
    node.setDatarate(DR_SF8);
    serial.printf("Set SF for SF9\n\r");
  } else if (counter < 3000) {
    node.setDatarate(DR_SF9);
    serial.printf("Set SF for SF12\n\r");
  } else if (counter < 4000) {
    node.setDatarate(DR_SF10);
    serial.printf("Set SF for SF12\n\r");
  } else if (counter < 5000) {
    node.setDatarate(DR_SF11);
    serial.printf("Set SF for SF12\n\r");
  } else if (counter < 6000) {
    node.setDatarate(DR_SF12);
    serial.printf("Set SF for SF12\n\r");
  } 
  else {
    node.setDatarate(DR_SF7);
    serial.printf("Set SF for SF7\n\r");
    counter = 0;
  }

  Int_calculada = med_Ieficaz();
  Serial.println(Int_calculada);
  Int_calculada = Int_calculada * multiplier; // é multiplicado por um valor de correção baseado em medições reais
  Peficaz = Int_calculada * VOLTRED;  //P=V*I

  Serial.printf("%.4f", Int_calculada);
  Serial.println(" A");
  Serial.print(Peficaz);
  Serial.println(" W");

  // Build payload byte array
  uint8_t uplinkPayload[242];
  char message[100];
  snprintf(message, sizeof(message), "c|%f", Int_calculada);
  serial.println(message);
  strcpy((char*) uplinkPayload, message);
  // Reseta contador do pluviômetro
  // Perform an uplink
  int state = node.sendReceive(uplinkPayload, strlen((char*) uplinkPayload));
  debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);
  counter++;
  // Check if a downlink was received 
  // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
  if(state > 0) {
    serial.println(F("Received a downlink"));
  } else {
    serial.println("No downlink received");
  }
  // Wait until next uplink - observing legal & TTN FUP constraints
  delay(uplinkIntervalSeconds * 1000);
}


float med_Ieficaz() {                         
  long tempoinicio = millis();                       // para medir quanto tempo leva para realizar as medições
  int16_t bitsads;
  float mVporbit = 0.0625F;
  float Ieficaz;
  float Iinstant;
  float mVinstant;
  float sumIinstant=0;

  for (int i = 0; i < CANTMUESTRAS; i++) {
    bitsads = ads.readADC_Differential_0_1();
    mVinstant = bitsads * mVporbit;
    Iinstant = mVinstant * AMAXSENS / MVMAXSENS;   // regra de três baseada no sensor conectado já que o sensor oferece tensão e a passamos diretamente proporcional à intensidade
    sumIinstant += sq(Iinstant);                   // soma dos quadrados
  }
  Ieficaz = sqrt(sumIinstant / CANTMUESTRAS);        // raiz quadrada da soma dos quadrados dividida pelo número de amostras

  long tempofim = millis();
  Serial.print((tempofim - tempoinicio) / 1000);
  Serial.println(" segundos para medir");
  return (Ieficaz);
}