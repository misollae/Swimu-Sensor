#include <base64.hpp>
#include <Utils.h>
#include <ArduinoBLE.h>
#include <LSM6DS3.h>      // https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3
#include <Wire.h>
#include "SD.h"
#include <SPI.h>
#include <avr/pgmspace.h>
#include <iostream>
#include <chrono>
#include <string>

//I2C device address 0x6A
const byte addressIMU = 0x6A;
LSM6DS3 lsm6ds33(I2C_MODE, addressIMU );
typedef struct {
} DataIMU, * PtrDataIMU;

DataIMU dataIMU;
long lastTime;
long lastInterval;
bool trainingOn;

/************************* LED Status *********************************/
const int LedRGBOnValue  = LOW;
const int LedRGBOffValue = HIGH;

/************************* Bluetooth *********************************/
// Service - Current Time Service (0x1805)
const char* serviceGATT = "1805";
const char* nameCentralBLE = "XIAO Swimu/Sport (CJG)";

// Time service
BLEService timeService( serviceGATT );

// Swimu/Sports characteristics
const char* characteristicCurrentTimeServiceGATT    = "2A2B";
const char* characteristicCurrentTimeServiceValue   = "Date Time";
const int characteristicCurrentTimeServiceValueSize = 10;

// Swimu/Sports characteristics
BLEStringCharacteristic characteristicCurrentTimeService( characteristicCurrentTimeServiceGATT, BLEWrite, 20 );
BLEDescriptor descriptorCurrentTimeService( characteristicCurrentTimeServiceGATT, characteristicCurrentTimeServiceValue );

/************************* SD Card *********************************/
// Variables
#define DATA_DIR "SWIMU/SESSIONS"
#define SD_PIN 3                           // PIN do cartão SD

float temperature, pressure, altitude;        
float magnetic_x, magnetic_y, magnetic_z;
float humidity;
float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second)
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)
uint8_t roll, pitch, yaw;
bool SDready;
char sessionFileName[23];

long lastSave = 0;

/************************* LED Status *********************************/
void initLED(const int ledPin) {
    pinMode( ledPin, OUTPUT );
    digitalWrite( ledPin, LedRGBOffValue );
}

void myBlink(const int ledPin, const int blinkTime) {
    digitalWrite(ledPin, LedRGBOnValue);
    delay( blinkTime );
    digitalWrite(ledPin, LedRGBOffValue);
    delay( blinkTime );
}

/************************* BLUETOOTH *********************************/
void setupBLE(){
    DebugMessagePrintf( "Device LSM6DS3 is ready. Going to calibrate IMU...\n" );

    if ( !BLE.begin() ) {
        DebugMessagePrintf( "Starting Bluetooth® Low Energy module failed!\n" );

        // Turn on the Red LED on to indicate that there was an error on setup
        while ( 1 ) {
            myBlink( LEDR, 1000 );
        }
    }

    DebugMessagePrintf( "Bluetooth® Low Energy is ready.\n" );
    DebugMessagePrintf( "Setting BLE Central name to: %s\n", nameCentralBLE );
    BLE.setLocalName( nameCentralBLE );
    DebugMessagePrintf( "Setting the advertise service...\n" );
    BLE.setAdvertisedService( timeService );
    DebugMessagePrintf( "Adding characteristics to service...\n" );
    timeService.addCharacteristic( characteristicCurrentTimeService );
    DebugMessagePrintf( "Adding descriptors to characteristic...\n" );
    characteristicCurrentTimeService.addDescriptor( descriptorCurrentTimeService );
    DebugMessagePrintf( "Preparing onWrite function...\n" );
    characteristicCurrentTimeService.setEventHandler(BLEWritten, onCurrentTimeServiceReceived);
    DebugMessagePrintf( "Adding service to BLE...\n" );
    BLE.addService( timeService );
    DebugMessagePrintf( "Going to advertise...\n" );
    BLE.advertise();
    DebugMessagePrintf( "Device MAC: %d\n", BLE.address().c_str() );
    DebugMessagePrintf( "Setup is ready.\n" );
    DebugMessagePrintf( "Waiting for connections in five seconds...\n\n" );
    digitalWrite(LEDG, LedRGBOnValue);
}

void onCurrentTimeServiceReceived(BLEDevice central, BLECharacteristic characteristic) {
  DebugMessagePrintf( "?");

  if (!trainingOn && characteristic == characteristicCurrentTimeService) {
    uint8_t value[20];
    int bytesRead = characteristic.readValue(value, 20);

    uint16_t year  = (value[1] << 8) | value[0];
    uint8_t month  = value[2];
    uint8_t day    = value[3];
    uint8_t hour   = value[4];
    uint8_t minute = value[5];
    uint8_t second = value[6];
    
    if (SDready) {
      sprintf(sessionFileName, "/%04u%02u%02u/%02u%02u%02u.txt", year, month, day, value[4], value[5], value[6]);
      DebugMessagePrintf( "String: %s \n", sessionFileName);
      char dateDir[9];
      sprintf(dateDir, "/%04u%02u%02u", year, month, day);
      prepareSession(dateDir);
      calibrateIMU( 250, 250);
      lastTime = millis();
    }
  }
}

/************************* SD CARD *********************************/
void setupSD(){
    SDready = false;

    DebugMessagePrintf("Initializing SD card...\n");
    if(!SD.begin(SD_PIN)) {
      DebugMessagePrintf("SD card initialization failed!\n\n");
    }

    DebugMessagePrintf("Card ready, checking for necessary directories...\n");
    if(!SD.exists(DATA_DIR)) {
      if(!SD.mkdir(DATA_DIR)) {
        DebugMessagePrintf("Create Directory failed\n");
      }
      File myFile = SD.open(String(DATA_DIR) + "/tosave.txt", FILE_WRITE);
      myFile.close();
    }

    SDready = true;
    Serial.println("All ready!\n");
}

void prepareSession(char* dateDir) {
  DebugMessagePrintf("\nNow starting session file\n")

  if(!SD.exists(String(DATA_DIR) + String(dateDir))) {
    SD.mkdir(String(DATA_DIR) + String(dateDir));
  }
  
  File sessionFile = SD.open(String(DATA_DIR) + String(sessionFileName), FILE_WRITE);

  if(!sessionFile) { 
      DebugMessagePrintf( "Session failed to start!\n" );
      while ( 1 ) {
          myBlink( LEDR, 500 );
      }
   }
  
  sessionFile.println("timestamp ; roll ; pitch ; yaw");
  sessionFile.close();

  File toSaveFile = SD.open(String(DATA_DIR) + "/tosave.txt", FILE_WRITE);

  if(!toSaveFile) { 
    DebugMessagePrintf( "Session failed to start!\n" );
      while ( 1 ) {
          myBlink( LEDR, 500 );
      }
  }
  toSaveFile.println(sessionFileName);
  toSaveFile.close();

  DebugMessagePrintf("\nTraining Started\n")
  myBlink( LEDB, 1000 );
  trainingOn = true;
}

void SaveCalculations(long timestamp) {

  File sessionFile = SD.open(String(DATA_DIR) + String(sessionFileName), FILE_WRITE);

  if(!sessionFile) { 
      DebugMessagePrintf( "Failed to save!\n" );
      while ( 1 ) {
          myBlink( LEDR, 500 );
      }
   }

  sessionFile.println(String(timestamp) + " ; " + String(roll) + " ; " + String(pitch) + " ; " + String(yaw));
  sessionFile.close();
  lastTime = timestamp;
}

/************************* OTHER *********************************/
void setup() {
    DebugDelay( 1000 );
    DebugSerialBeginNoBlock( 115200 );

    trainingOn = false;

    DebugMessagePrintf( "Serial port is ready.\n" );
    DebugMessagePrintf( "Configuring Builtin LED...\n" );
    initLED( LED_BUILTIN );
    DebugMessagePrintf( "Configuring RGB LED...\n" );
    initLED( LEDR );
    initLED( LEDG );
    initLED( LEDB );
    
    if ( lsm6ds33.begin()!=0 ) {
        DebugMessagePrintf( "Starting device LSM6DS3 failed!\n" );
        while ( 1 ) {
            myBlink( LEDR, 500 );
        }
    }

    setupSD();
    setupBLE();
}

bool readIMU() {
  accelX = lsm6ds33.readFloatAccelX();
  accelY = lsm6ds33.readFloatAccelY();
  accelZ = lsm6ds33.readFloatAccelZ();
  
  gyroX = lsm6ds33.readFloatGyroX();
  gyroY = lsm6ds33.readFloatGyroY();
  gyroZ = lsm6ds33.readFloatGyroZ();
  
  return true;
}

void calibrateIMU(int delayMillis, int calibrationMillis) {
  int calibrationCount = 0;

  delay(delayMillis); // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    }
  }

  if (calibrationCount == 0) {
    DebugMessagePrintf( "Failed to calibrate!\n" );
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;
}

void doImuCalculations() {
  accRoll = atan2(accelY, accelZ) * 180 / M_PI;
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;

  float lastFrequency = (float) 1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll  = gyroCorrectedRoll  + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw   = gyroCorrectedYaw   + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll  = complementaryRoll  + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw   = complementaryYaw   + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll  = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;

  roll = map(complementaryRoll, -180, 180, 0, 255);
  pitch = map(complementaryPitch, -180, 180, 0, 255);
  yaw = map(complementaryYaw, -180, 180, 0, 255);
}

void loop() {
    // Wait for a BLE central to connect
    BLEDevice central = BLE.central();

    // If central is connected to peripheral
    if ( central ) {
        DebugMessagePrintf( "Connected to peripheral device MAC: %s\n", central.address().c_str() );

        // Turn on the Blue LED on to indicate the connection
        digitalWrite(LEDB, LedRGBOnValue);
        
        while ( central.connected() ) {
          BLE.poll();

          if ( readIMU() && trainingOn) {
            long currentTime = micros();
            lastInterval = currentTime - lastTime;
            if (lastInterval > 2000) {
              doImuCalculations();
              SaveCalculations(currentTime);
            }
          }
        }

        // Turn off the Blue LED to indicate lost of connection
        digitalWrite(LEDB, LedRGBOffValue );
        DebugMessagePrintf( "Disconnected from central MAC: %s\n", central.address().c_str() );
    }
}
