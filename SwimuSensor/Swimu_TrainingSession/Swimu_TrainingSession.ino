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
#include "Display.h"
#include <stdlib.h>

//I2C device address 0x6A
const byte addressIMU = 0x6A;
LSM6DS3 lsm6ds33(I2C_MODE, addressIMU );
typedef struct {
} DataIMU, * PtrDataIMU;

DataIMU dataIMU;
long lastTime;
long startTime;
long lastInterval;
long lastFlushTime;

// The sensor has 4 possible states
//   . Waiting         > "Has no work", waiting for either a BLE or USB connection;
//   . bleConnected    > Is connected via BLE to a device, waiting for orders;
//   . savingSession   > Is printing calculations into a session file;
//   . sessionTransfer > Is undergoing a FTP via USB with mobile device;
enum {waiting, savingSession, sessionTransfer};
unsigned char state = waiting; // What the sensor is doing

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

String sessionFileName;
File sessionFile;
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
int roll, pitch, yaw;
bool SDready;

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
  DebugMessagePrintf("\nReceived\n");

  if (characteristic == characteristicCurrentTimeService) {
    if (state == savingSession) {
      endSession();
    } else {
      uint8_t value[20];
      int bytesRead = characteristic.readValue(value, 20);

      if (SDready) {
        char fileDir[9];
        char fileName[7];
        sprintf(fileDir, "/%04u%02u%02u", (value[1] << 8) | value[0], value[2], value[3]);
        sprintf(fileName, "/%02u%02u%02u", value[4], value[5], value[6]);
        prepareSession(fileDir, fileName);
      }
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
      myFile.print("0\n0");
      myFile.close();
    }

    //if(SD.rmdir(String(DATA_DIR) + "/20230419")) DebugMessagePrintf("rem\n");

    SDready = true;
    Serial.println("All ready!\n");
}

void prepareSession(char* fileDir, char* fileName) {
  if(!SD.exists(String(DATA_DIR) + String(fileDir))) {
    SD.mkdir(String(DATA_DIR) + String(fileDir));
  }
  sessionFileName = String(fileDir) + String(fileName) + ".txt";
  sessionFile = SD.open(String(DATA_DIR) + String(fileDir) + String(fileName) + ".txt", FILE_WRITE);
  
  if(sessionFile) { 
    sessionFile.println("timestamp ; roll ; pitch ; yaw");
    DebugMessagePrintf("\nStarted current session\n")
    calibrateIMU( 250, 250);
    digitalWrite(LEDB, LedRGBOnValue);
    startTime = micros();
    lastFlushTime = 0;
    state = savingSession;
    return;
  }

  DebugMessagePrintf( "Session failed to start!\n" );
  digitalWrite(LEDB, LedRGBOffValue );
  state = waiting;
}

void endSession() {
  sessionFile.close();
  DebugMessagePrintf( "Ended current session");
  digitalWrite(LEDB, LedRGBOffValue );
  state = waiting;
  return;
}

void SaveCalculations(long timestamp) {

  if(!sessionFile) { 
      DebugMessagePrintf( "Failed to save!\n" );
      myBlink( LEDR, 500 );
   } else {
      unsigned long saveMillis = (unsigned long)((timestamp  - startTime) / 1000);   
      sessionFile.print("\n"); 
      sessionFile.print(String(saveMillis));
      sessionFile.print(';');
      sessionFile.print(String(roll));
      sessionFile.print(';');
      sessionFile.print(String(pitch));
      sessionFile.print(';');
      sessionFile.println(String(yaw));

      //DebugMessagePrintf(String(saveMillis) + " ; " + String(roll) + " ; " + String(pitch) + " ; " + String(yaw) + "\n");

      if (saveMillis - lastFlushTime >= 60000) {
        sessionFile.flush();
        DebugMessagePrintf( "\nFlushed!\n" );
        lastFlushTime = saveMillis;
      }
   }
   
}

void sendFileList(){
  File sessionsRoot = SD.open(String(DATA_DIR));
  if (!sessionsRoot) {
    DebugMessagePrintf( "Failed to open root!\n" );
    return;
  }

  DebugMessagePrintf( "Files:\n" );
  sendFolderFileList(sessionsRoot, "");
  Serial.println("End of list");
}

void sendFolderFileList(File dir, String parentDir) {
  while (true) {
    File file = dir.openNextFile();
    if (!file) {
      break;
    }

    if (file.isDirectory()) {
      String currentDir = parentDir + "/" + file.name();
      sendFolderFileList(file, currentDir);
    } else if (strcmp(file.name(), "TOSAVE.TXT") != 0) {
      String pathName = parentDir + "/" + file.name();
      Serial.println(pathName.substring(0, pathName.lastIndexOf(".")));
    }
    file.close();
  }
}

bool transferNextFile(String fileName){
  File toTransferFile = SD.open(String(DATA_DIR) + String(fileName) + ".txt");
  if(toTransferFile) {  
    size_t bufferSize = 1024; 
    uint8_t buffer[bufferSize];
    size_t bytesRead;
    
    do {
      bytesRead = toTransferFile.read(buffer, bufferSize);

      if (buffer[0] == '\n' || buffer[0] == '\r') Serial.write(" ");

      Serial.write(buffer, bytesRead);
      if (buffer[bytesRead - 1] == '\n' || buffer[bytesRead - 1] == '\r') Serial.write(" ");

    } while (toTransferFile.available() || bytesRead > 0);

    toTransferFile.close();
    Serial.println("End of file");
    SD.remove(String(DATA_DIR) + String(fileName) + ".txt");
    return true;
  }
  return false;
}

/************************* OTHER *********************************/
void setup() {
    initializeDisplay();   
    clearDisplay(2000);
    DebugSerialBeginNoBlock( 115200 );

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
  if ( complementaryYaw<0.0f ) {
    complementaryYaw = complementaryYaw + 360.0f;
  }
  else {
    if ( complementaryYaw>360.0f ) {
      complementaryYaw = complementaryYaw - 360.0f;
    }
  }


  roll  = map(complementaryRoll, -180, 180, 0, 360);
  pitch = map(complementaryPitch, -180, 180, 0, 360);
  yaw   = (int)complementaryYaw;
}

int i = 0;
void loop() {
  BLEDevice central = BLE.central();
  switch (state) {  
    
    case waiting:

      if ( central ) {
        if ( central.connected() ) BLE.poll();
      }
      
      if (Serial.available()) {
        state = sessionTransfer;
      }

      break;

    case savingSession:
      if ( central ) {
        if ( central.connected() ) BLE.poll();
      }

      if (readIMU()) {
        long currentTime = micros();
        lastInterval = currentTime - lastTime;
        lastTime = currentTime;
        doImuCalculations();
        //DebugMessagePrintf("\n" + String(roll) + " " + String(pitch) + " " + String(yaw) + "\n");
        SaveCalculations(currentTime);
      } 


      break;

    case sessionTransfer:

      if (Serial.available()) {
        String message = Serial.readStringUntil('\n');

        if (message.equals("Show file list")) { 
          sendFileList();
        } else if (message.length() == 16) {
          myBlink( LEDB, 500 );
          transferNextFile(message);
        } else if (message.equals("End transfer")) { 
          state = waiting;
        }
      }      

      if (!Serial) {
        DebugMessagePrintf( "USB disconnected" );
        state = waiting;
      }
      break;

    default:
      break;
  }
}

