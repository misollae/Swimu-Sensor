/*********
  Abril 2023
*********/
// Libs
#include "SD.h"
#include <SPI.h>
#include <Utils.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <iostream>
#include <chrono>
#include <string>

// Variables
#define DATA_DIR "SWIMU/SAVED"
#define SD_PIN 3                           // PIN do cartão SD

LSM6DS3 lsm6ds33(I2C_MODE, 0x6A);          // Relativos ao sensor...
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
long lastInterval;

bool SDready;

String sessionFileName;

long lastTime;
long lastSave = 0;

/***************************************************************************************************************************/
void setup() {
    DebugDelay( 10000 );
    DebugSerialBegin( 115200 ); 
    SDready = false;

    if ( lsm6ds33.begin()!=0 ) {
      DebugMessagePrintf( "Device error!\n" );
    }
    else {
      DebugMessagePrintf( "Device OK.\n" );
    }

    // Cartão de Memória
    SDready = initSDCard();

    if (SDready) {
      prepareSession();
      calibrateIMU( 250, 250);
      lastTime = millis();
      DebugMessagePrintf( "Setup ready.\n" );      
    }
}


/******** FILE SYSTEM RELATED METHODS ***************************************/
bool initSDCard(){  
  DebugMessagePrintf("Initializing SD card...\n");
  
  if(!SD.begin(SD_PIN)) {
    DebugMessagePrintf("SD card initialization failed!\n\n");
    return false;
  }

  DebugMessagePrintf("Card ready, checking for necessary directories...\n");
  if(!SD.exists(DATA_DIR)) {
    if(!SD.mkdir(DATA_DIR)) {
      DebugMessagePrintf("Create Directory failed\n");
      return false;
    }
    File myFile = SD.open(String(DATA_DIR) + "/next.txt", FILE_WRITE);
    myFile.close();
  }
  
  Serial.println("All ready!\n");
  return true;
}

void prepareSession() {
  sessionFileName = "/teste.txt";  

  File sessionFile = SD.open(String(DATA_DIR) + String(sessionFileName), FILE_WRITE);
  DebugMessagePrintf("\nNow starting session file\n")
  if(!sessionFile) { return; }
  
  sessionFile.println("timestamp ; roll ; pitch ; yaw");
  sessionFile.close();
}

void SaveCalculations(long timestamp) {

  File sessionFile = SD.open(String(DATA_DIR) + String(sessionFileName), FILE_WRITE);

  if(!sessionFile) { return; }

  sessionFile.println(String(timestamp) + " ; " + String(roll) + " ; " + String(pitch) + " ; " + String(yaw));
  sessionFile.close();
  lastTime = timestamp;
}

/******** DATA RELATED METHODS ***************************************/

bool readIMU() {
  accelX = lsm6ds33.readFloatAccelX();
  accelY = lsm6ds33.readFloatAccelY();
  accelZ = lsm6ds33.readFloatAccelZ();
  
  gyroX = lsm6ds33.readFloatGyroX();
  gyroY = lsm6ds33.readFloatGyroY();
  gyroZ = lsm6ds33.readFloatGyroZ();
  
  return true;
}

/* 
  the gyro's x,y,z values drift by a steady amount. if we measure this when arduino is still
  we can correct the drift when doing real measurements later
*/
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
  if ( readIMU() && SDready) {
    long currentTime = micros();
    lastInterval = currentTime - lastTime;
    if (lastInterval > 2000) {
      doImuCalculations();
      SaveCalculations(currentTime);
    }
  }
}
