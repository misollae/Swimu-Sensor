// https://gist.githubusercontent.com/jasoncoon/926f037e72ec392d164f11d263db68ac/raw/18cfbf4843d56ddb3d30553f5dde45038cd0abf1/fibonacci64-xiao-ble-sense-yaw.ino
// Fibonacci64 - Seeed XIAO BLE Sense - Arduino & NeoPixel
// https://gist.github.com/jasoncoon/926f037e72ec392d164f11d263db68ac
// https://www.evilgeniuslabs.org/fibonacci64-nano

// Adapted from Adafruit's Arduino Sensor Example: https://learn.adafruit.com/adafruit-feather-sense/arduino-sensor-example
// and owennewo's roll pitch yaw example: // https://github.com/arduino-libraries/Arduino_LSM6DS3/blob/5eac7f5e6145c4747da27698faf3a548d2893a2b/examples/RollPitchYaw/RollPitchYaw.ino

#include <Utils.h>

#include <ArduinoBLE.h>

#include <LSM6DS3.h> // https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3
#include <Wire.h>

const int LedRGBOnValue = LOW;
const int LedRGBOffValue = HIGH;

//I2C device address 0x6A
const byte addressIMU = 0x6A;

LSM6DS3 lsm6ds33(I2C_MODE, addressIMU );

typedef struct {
} DataIMU, * PtrDataIMU;

DataIMU dataIMU;

long lastTime;
long lastInterval;

// Service - Current Time Service (0x1805)
const char* serviceGATT = "1805";
const char* nameCentralBLE = "XIAO Swimu/Sport (CJG)";

// Time service
BLEService timeService( serviceGATT );

// Swimu/Sports characteristics
const char* characteristicCurrentTimeServiceGATT = "2A2B";
const char* characteristicCurrentTimeServiceValue = "Epoch Time (millis)";
const int characteristicCurrentTimeServiceValueSize = strlen( characteristicCurrentTimeServiceValue ) + 1;

// Swimu/Sports characteristics
BLEByteCharacteristic characteristicCurrentTimeService( characteristicCurrentTimeServiceGATT, BLERead );
BLEDescriptor descriptorCurrentTimeService( characteristicCurrentTimeServiceGATT, characteristicCurrentTimeServiceValue );

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

void onCurrentTimeServiceReceived(BLEDevice central, BLECharacteristic characteristic) {
  if (characteristic == characteristicCurrentTimeService) {
    char value[characteristicCurrentTimeServiceValueSize];
    int bytesRead = characteristic.readValue(value, characteristicCurrentTimeServiceValueSize);
    Serial.print("Received value: ");
    Serial.println(value);
  }
}

void setup() {
    DebugDelay( 1000 );

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

        // Turn on the Red LED on to indicate that there was an error on setup
        while ( 1 ) {
            myBlink( LEDR, 500 );
        }
    }
    
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

void loop() {
    // Wait for a BLE central to connect
    BLEDevice central = BLE.central();

    // If central is connected to peripheral
    if ( central ) {
        DebugMessagePrintf( "Connected to peripheral device MAC: %s\n", central.address().c_str() );

        // Turn on the Blue LED on to indicate the connection
        digitalWrite(LEDB, LedRGBOnValue);
        
        while ( central.connected() ) {
        }

        // Turn off the Blue LED to indicate lost of connection
        digitalWrite(LEDB, LedRGBOffValue );

        DebugMessagePrintf( "Disconnected from central MAC: %s\n", central.address().c_str() );
    }
}
