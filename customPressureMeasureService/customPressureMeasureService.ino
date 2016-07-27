/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
    Please note the long strings of data sent mean the *RTS* pin is
    required with UART to slow down data sent to the Bluefruit LE!
*/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"


#define BLUEFRUIT_HWSERIAL_NAME      Serial1

#define BLUEFRUIT_VENTIL_PIN             9    // we won't be changing modes
#define BLUEFRUIT_PUMP_PIN             6    // we won't be changing modes
#define BLUEFRUIT_UART_MODE_PIN        12    // we won't be changing modes
#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
 //Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */

int32_t pressureServiceId;
int32_t pressureReadableCharId;
int32_t pressureStrengthWritableCharId;
int32_t pressurePatternWritableCharId;
int32_t pressureValueCharId;
int32_t pressureFakeCharId;
int sensorPin=A4;
int sensorValue=0;

long int pattern;
long int strength;
/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(BLUEFRUIT_PUMP_PIN, OUTPUT);
  pinMode(BLUEFRUIT_VENTIL_PIN, OUTPUT);
  analogWrite(BLUEFRUIT_PUMP_PIN, 0);
  digitalWrite(BLUEFRUIT_VENTIL_PIN, LOW);
  //while (!Serial); // required for Flora & Micro
  delay(500);

  boolean success;
/*
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Pressure Measure"));
  Serial.println(F("---------------------------------------------------"));
*/
  randomSeed(micros());

  /* Initialise the module */
  //Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  //Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  //Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  //Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  //ble.info();

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  //ble.setInterCharWriteDelay(5); // 5 ms

  /* Add the Pressure Service definition */
  /* Service ID should be 1 */
  //Serial.println(F("Adding custom Pressure Service definition: "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID128=00-77-13-12-11-00-00-00-00-00-AB-BA-0F-A1-AF-E1"), &pressureServiceId);
  if (! success) {
    error(F("Could not add Pressure service"));
  }

  /* Add the readable pressure characteristic */
  /* Chars ID for Measurement should be 1 */
  //Serial.println(F("Adding the readable pressure characteristic: "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=00-69-42-03-00-77-12-10-13-42-AB-BA-0F-A1-AF-E1,PROPERTIES=0x02,MIN_LEN=1, VALUE=0"), &pressureReadableCharId);
    if (! success) {
    error(F("Could not add readable pressure characteristic"));
  }
  
  /* Add the writable pressure strength characteristic */
  /* Chars ID for Measurement should be 2 */
  //Serial.println(F("Adding the readable pressure characteristic: "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=00-70-42-04-00-77-12-10-13-42-AB-BA-0F-A1-AF-E1,PROPERTIES=0x08,MIN_LEN=1, VALUE=0"), &pressurePatternWritableCharId);
    if (! success) {
    error(F("Could not add writable pressure characteristic"));
  }

  /* Add the writable pressure pattern characteristic */
  /* Chars ID for Measurement should be 3 */
  //Serial.println(F("Adding the readable pressure characteristic: "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=00-71-42-04-00-77-12-10-13-42-AB-BA-0F-A1-AF-E1,PROPERTIES=0x08,MIN_LEN=1, VALUE=0"), &pressureStrengthWritableCharId);
    if (! success) {
    error(F("Could not add writable pressure characteristic"));
  }

  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=00-72-42-04-00-77-12-10-13-42-AB-BA-0F-A1-AF-E1,PROPERTIES=0x02,MIN_LEN=1, VALUE=0"), &pressureValueCharId);
    if (! success) {
    error(F("Could not add writable pressure characteristic"));
  }

  /* Add the custom Pressure Service to the advertising data (needed for Nordic apps to detect the service) */
  //Serial.print(F("Adding Pressure Service UUID to the advertising payload: "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-03-02-12-13") );

  /* Reset the device for the new service setting changes to take effect */
  //Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();
}

void loop(void)
{
  ble.println("AT+GATTCHAR=2");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  String bufferString=ble.buffer;
  pattern = strtol(bufferString.c_str(),NULL,0);

  ble.println("AT+GATTCHAR=3");
  ble.readline();
  
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  bufferString=ble.buffer;
  strength = strtol(bufferString.c_str(),NULL,0)*10;
  if(strength!=0){
    ble.sendCommandCheckOK( F("AT+GATTCHAR=1,1") );
  }

  if (pattern != 0){
    notifyAboutNotifications(pattern, strength);
  }
  delay(10);

}

void notifyAboutNotifications(long int pattern, long int strength)
{
  digitalWrite(BLUEFRUIT_VENTIL_PIN, HIGH);
  ble.sendCommandCheckOK( F("AT+GATTCHAR=1,2") );
  if(pattern==1){
    pumpAir(strength);
  }else if(pattern==2){
    delay(strength*100);
  }else if(pattern==3){
    releaseAir(strength);
  }
  else if(pattern==4){
    releaseWhilePumping(strength);
  }
  else if(pattern==6){
    sinusPumping;
  }
  ble.sendCommandCheckOK( F("AT+GATTCHAR=1,0") );
  ble.sendCommandCheckOK( F("AT+GATTCHAR=2,0") );
  ble.sendCommandCheckOK( F("AT+GATTCHAR=3,0") );
  
}


void pumpAir(int desiredPressure){
  analogWrite(BLUEFRUIT_PUMP_PIN, 127);
  int limit=0;
  while(sensorValue<desiredPressure && limit<30000){
    limit+=10;
    sensorValue=analogRead(sensorPin);
    Serial.println( sensorValue );
    delay(10);
  }
  analogWrite(BLUEFRUIT_PUMP_PIN, 0);
}

void sinusPumping(){
  analogWrite(BLUEFRUIT_PUMP_PIN, 127);
  while(pattern!=7){

    ble.println("AT+GATTCHAR=2");
    ble.readline();
    String bufferString=ble.buffer;
    pattern = strtol(bufferString.c_str(),NULL,0);

    sensorValue=analogRead(sensorPin);
    if(sensorValue>=1000){
      digitalWrite(BLUEFRUIT_VENTIL_PIN, LOW);
      while(sensorValue>=200 && pattern!=7){
        ble.println("AT+GATTCHAR=2");
        ble.readline();
        bufferString=ble.buffer;
        pattern = strtol(bufferString.c_str(),NULL,0);
        sensorValue=analogRead(sensorPin);
        delay(10);
      }
      digitalWrite(BLUEFRUIT_VENTIL_PIN, HIGH);
    }
    delay(10);
  }
  digitalWrite(BLUEFRUIT_VENTIL_PIN, LOW);
  analogWrite(BLUEFRUIT_PUMP_PIN, 0);
}

void releaseWhilePumping(int desiredPressure){
  analogWrite(BLUEFRUIT_PUMP_PIN, 127);
  digitalWrite(BLUEFRUIT_VENTIL_PIN, LOW);
  int limit=0;
  while(sensorValue>desiredPressure && limit<30000){
    limit+=10;
    sensorValue=analogRead(sensorPin);
    Serial.println( sensorValue );
    delay(10);
  }
  analogWrite(BLUEFRUIT_PUMP_PIN, 0);
  if(sensorValue>110){
    digitalWrite(BLUEFRUIT_VENTIL_PIN, HIGH);
  }
}

void releaseAir(int desiredPressure){
  digitalWrite(BLUEFRUIT_VENTIL_PIN, LOW);
  while(sensorValue>desiredPressure){
    sensorValue=analogRead(sensorPin);
    delay(10);
  }
  if(sensorValue>110){
    digitalWrite(BLUEFRUIT_VENTIL_PIN, HIGH);
  }
}


