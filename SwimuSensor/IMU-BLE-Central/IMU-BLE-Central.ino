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
    // units m/s/s i.e. accelZ if often 9.8 (gravity)
    float accelX;
    float accelY;
    float accelZ;

    // units dps (degrees per second)
    float gyroX;
    float gyroY;
    float gyroZ;

    // units dps
    float gyroDriftX;
    float gyroDriftY;
    float gyroDriftZ;

    // units degrees (expect major drift)
    float gyroRoll;
    float gyroPitch;
    float gyroYaw;

    // units degrees (expect minor drift)
    float gyroCorrectedRoll;
    float gyroCorrectedPitch;
    float gyroCorrectedYaw;

    // units degrees (roll and pitch noisy, yaw not possible)
    float accRoll;
    float accPitch;
    float accYaw;

    // units degrees (excellent roll, pitch, yaw minor drift)
    float complementaryRoll;
    float complementaryPitch;
    float complementaryYaw;

    uint8_t roll, pitch, yaw;
} DataIMU, * PtrDataIMU;

DataIMU dataIMU;

long lastTime;
long lastInterval;

// Service - Physical Activity Monitor service (0x183E)
const char* serviceGATT = "183E";

// Characteristic - Device Wearing Position (0x2B4B)
const char* characteristicDevicWearingPositionGATT = "2B4B";
const char* characteristicDevicWearingPositionDescriptionValue = "Sensor location";
const int characteristicDevicWearingPositionDescriptionValueSize = strlen( characteristicDevicWearingPositionDescriptionValue ) + 1;
/*
0x00  Other
0x01  Head 
0x02  Head_Ear 
0x03  Head_Ear_Right 
0x04  Head_Ear_Left 
0x05  Head_Neck 
0x06  Trunk 
0x07  Trunk_Pelvis 
0x08  Trunk_Pelvis_Right 
0x09  Trunk_Pelvis_Left 
0x0A  Trunk_Thorax 
0x0B  Trunk_Thorax_Right 
0x0C  Trunk_Thorax_Left 
0x0D  Trunk_Back
...
0x1A  LowerExtremity 
0x1B  LowerExtremity_Right 
0x1C  LowerExtremity_Left 
0x1D  LowerExtremity_Ankle 
0x1E  LowerExtremity_Ankle_Right 
0x1F  LowerExtremity_Ankle_Left 
0x20  LowerExtremity_Foot 
0x21  LowerExtremity_Foot_Right 
0x22  LowerExtremity_Foot_Left 
0x23  Pants_Pocket 
0x24  Pants_Pocket_Right 
0x25  Pants_Pocket_Left 
0x26  Chest_Pocket 
0x27  Chest_Pocket_Right 
0x28  Chest_Pocket_Left 
0x29–0xFF  Reserved for Future Use 
*/

// Characteristic - General Activity Instantaneous Data (0x2B3C)
const char* characteristicGeneralActivityInstantaneousDataGATT = "2B3C";

// Characteristic User Description (0x2901)
const char* characteristicUserDescriptionGATT = "2901";
const char* characteristicUserDescriptionDescriptionValue = "Roll(deg/sec), Pitch(deg/sec), Yaw(deg/sec), Frequency(Hz)";
const int characteristicUserDescriptionDescriptionValueSize = strlen( characteristicUserDescriptionDescriptionValue ) + 1;

/*
const char* characteristicDescriptionRollValue = "Roll";
const int characteristicDescriptionRollSize = strlen( characteristicDescriptionRollValue ) + 1;

const char* characteristicDescriptionPitchValue = "Pitch";
const int characteristicDescriptionPitchSize = strlen( characteristicDescriptionPitchValue ) + 1;

const char* characteristicDescriptionYawValue = "Yaw";
const int characteristicDescriptionYawSize = strlen( characteristicDescriptionYawValue ) + 1;

const char* characteristicDescriptionFrequencyValue = "Frequency";
const int characteristicDescriptionFrequencySize = strlen( characteristicDescriptionFrequencyValue ) + 1;
*/

const char* nameCentralBLE = "XIAO Swimu/Sport (CJG)";

// Swimu/Sports service
BLEService sportsService( serviceGATT );

// Swimu/Sports characteristics
BLEByteCharacteristic characteristicDevicWearingPosition( 
    characteristicDevicWearingPositionGATT, 
    BLERead | BLEWrite | BLENotify );
BLEDescriptor descriptorDevicWearingPosition( characteristicDevicWearingPositionGATT, characteristicDevicWearingPositionDescriptionValue );

BLEIntCharacteristic characteristicGeneralActivityInstantaneousDataRoll( 
    characteristicGeneralActivityInstantaneousDataGATT, 
    BLERead | BLENotify );
BLEIntCharacteristic characteristicGeneralActivityInstantaneousDataPitch( 
    characteristicGeneralActivityInstantaneousDataGATT, 
    BLERead | BLENotify );
BLEIntCharacteristic characteristicGeneralActivityInstantaneousDataYaw( 
    characteristicGeneralActivityInstantaneousDataGATT, 
    BLERead | BLENotify );
BLEFloatCharacteristic characteristicGeneralActivityInstantaneousDataFrequency( 
    characteristicGeneralActivityInstantaneousDataGATT, 
    BLERead | BLENotify );

BLEStringCharacteristic characteristicUserDescription(
    characteristicUserDescriptionGATT,
    BLERead | BLENotify ,
    characteristicUserDescriptionDescriptionValueSize);
BLEDescriptor descriptorUserDescription( characteristicUserDescriptionGATT, characteristicUserDescriptionDescriptionValue );

bool readIMU(DataIMU& imu) {
    imu.accelX = lsm6ds33.readFloatAccelX();
    imu.accelY = lsm6ds33.readFloatAccelY();
    imu.accelZ = lsm6ds33.readFloatAccelZ();
    
    imu.gyroX = lsm6ds33.readFloatGyroX();
    imu.gyroY = lsm6ds33.readFloatGyroY();
    imu.gyroZ = lsm6ds33.readFloatGyroZ();
    
    return true;
}

/*
  the gyro's x,y,z values drift by a steady amount. if we measure this when arduino is still
  we can correct the drift when doing real measurements later
*/
void calibrateIMU(DataIMU& imu, int delayMillis, int calibrationMillis) {
    int calibrationCount = 0;

    // to avoid shakes after pressing reset button
    delay( delayMillis ); 

    float sumX, sumY, sumZ;
    int startTime = millis();
    while ( millis() < (startTime + calibrationMillis) ) {
        if ( readIMU( imu ) ) {
            // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
            sumX += imu.gyroX;
            sumY += imu.gyroY;
            sumZ += imu.gyroZ;

            ++calibrationCount;
        }
    }

    if ( calibrationCount == 0 ) {
        DebugMessagePrintf( "Failed to calibrate!\n" );
    }

    imu.gyroDriftX = sumX / calibrationCount;
    imu.gyroDriftY = sumY / calibrationCount;
    imu.gyroDriftZ = sumZ / calibrationCount;
}

void doImuCalculations(DataIMU& imu) {
    imu.accRoll = atan2( imu.accelY, imu.accelZ) * 180 / M_PI;
    imu.accPitch = atan2( -imu.accelX, sqrt( imu.accelY * imu.accelY + imu.accelZ * imu.accelZ ) ) * 180 / M_PI;
    
    float lastFrequency = (float) 1000000.0 / lastInterval;
    imu.gyroRoll = imu.gyroRoll + ( imu.gyroX / lastFrequency );
    imu.gyroPitch = imu.gyroPitch + ( imu.gyroY / lastFrequency );
    imu.gyroYaw = imu.gyroYaw + ( imu.gyroZ / lastFrequency );
    
    imu.gyroCorrectedRoll = imu.gyroCorrectedRoll + ( ( imu.gyroX - imu.gyroDriftX ) / lastFrequency );
    imu.gyroCorrectedPitch = imu.gyroCorrectedPitch + ( ( imu.gyroY - imu.gyroDriftY) / lastFrequency );
    imu.gyroCorrectedYaw = imu.gyroCorrectedYaw + ( ( imu.gyroZ - imu.gyroDriftZ ) / lastFrequency );
    
    imu.complementaryRoll = imu.complementaryRoll + ( ( imu.gyroX - imu.gyroDriftX ) / lastFrequency );
    imu.complementaryPitch = imu.complementaryPitch + ( ( imu.gyroY - imu.gyroDriftY ) / lastFrequency );
    imu.complementaryYaw = imu.complementaryYaw + ( ( imu.gyroZ - imu.gyroDriftZ ) / lastFrequency );
    
    imu.complementaryRoll = 0.98 * imu.complementaryRoll + 0.02 * imu.accRoll;
    imu.complementaryPitch = 0.98 * imu.complementaryPitch + 0.02 * imu.accPitch;
    
    imu.roll = map( imu.complementaryRoll, -180, 180, 0, 255);
    imu.pitch = map( imu.complementaryPitch, -180, 180, 0, 255);
    imu.yaw = map( imu.complementaryYaw, -180, 180, 0, 255);
}

/**
   This comma separated format is best 'viewed' using 'serial plotter' or processing.org client (see https://github.com/arduino-libraries/Arduino_LSM6DS3/blob/5eac7f5e6145c4747da27698faf3a548d2893a2b/examples/RollPitchYaw/data/processing/RollPitchYaw3d.pde)
*/
void printCalculations(DataIMU& imu, long period) {
    //  Serial.print( imu.gyroRoll );
    //  Serial.print( ',' );
    //  Serial.print( imu.gyroPitch );
    //  Serial.print( ',' );
    //  Serial.print( imu.gyroYaw );
    //  Serial.print( ',' );
    //  Serial.print( imu.gyroCorrectedRoll );
    //  Serial.print( ',' );
    //  Serial.print( imu.gyroCorrectedPitch );
    //  Serial.print( ',' );
    //  Serial.print( imu.gyroCorrectedYaw );
    //  Serial.print( ', ');
    //  Serial.print( imu.accRoll );
    //  Serial.print( ',' );
    //  Serial.print( imu.accPitch );
    //  Serial.print( ',' );
    //  Serial.print( imu.accYaw );
    //  Serial.print( ',' );
    
    //  Serial.print( imu.complementaryRoll );
    //  Serial.print( ',');
    //  Serial.print( imu.complementaryPitch );
    //  Serial.print( ',' );
    //  Serial.print( imu.complementaryYaw );
    //  Serial.println( "" );

    //DebugMessagePrintf( "%d,%d,%d\n", imu.roll, imu.pitch, imu.yaw );
    //char* msg = format( "roll=%d,pitch=%d,yaw=%d,frequency=%5.2f Hz\n", imu.roll, imu.pitch, imu.yaw, 1.0/(period*0.000001) );
    //characteristicGeneralActivityInstantaneousData.setValue( msg );
    //delete msg;

    float frequency = 1.0f / ( period * 0.000001f );

    characteristicGeneralActivityInstantaneousDataRoll.writeValue( imu.roll );
    characteristicGeneralActivityInstantaneousDataPitch.writeValue( imu.pitch );
    characteristicGeneralActivityInstantaneousDataYaw.writeValue( imu.yaw );
    characteristicGeneralActivityInstantaneousDataFrequency.writeValue( frequency );
    
    DebugMessagePrintf( "roll=%d,pitch=%d,yaw=%d,frequency=%5.2f Hz\n", imu.roll, imu.pitch, imu.yaw, frequency );
}

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

    calibrateIMU( dataIMU, 250, 250 );

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
    BLE.setAdvertisedService( sportsService );

    DebugMessagePrintf( "Setting the advertise service...\n" );
    BLE.setAdvertisedService( sportsService );

    DebugMessagePrintf( "Adding characteristics to service...\n" );
    sportsService.addCharacteristic( characteristicDevicWearingPosition );
    sportsService.addCharacteristic( characteristicGeneralActivityInstantaneousDataRoll );
    sportsService.addCharacteristic( characteristicGeneralActivityInstantaneousDataPitch );
    sportsService.addCharacteristic( characteristicGeneralActivityInstantaneousDataYaw );
    sportsService.addCharacteristic( characteristicGeneralActivityInstantaneousDataFrequency );
    sportsService.addCharacteristic( characteristicUserDescription );

    DebugMessagePrintf( "Adding descriptors to characteristic...\n" );
    characteristicDevicWearingPosition.addDescriptor( descriptorDevicWearingPosition );
    characteristicUserDescription.addDescriptor( descriptorUserDescription );

    DebugMessagePrintf( "Adding service to BLE...\n" );
    BLE.addService( sportsService );

    DebugMessagePrintf( "Seeting initial values to characteristics ...\n" );
    characteristicDevicWearingPosition.writeValue( 0x07 /*Trunk_Pelvis*/ );
    characteristicUserDescription.setValue( characteristicUserDescriptionDescriptionValue );

    DebugMessagePrintf( "Going to advertise...\n" );
    BLE.advertise();

    DebugMessagePrintf( "Device MAC: %d\n", BLE.address().c_str() );

    DebugMessagePrintf( "Setup is ready.\n" );
    
    DebugMessagePrintf( "Waiting for connections in five seconds...\n\n" );

    // Turn on the Green LED on to indicate that setup is ready
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
            updateWrittings();
        }

        // Turn off the Blue LED to indicate lost of connection
        digitalWrite(LEDB, LedRGBOffValue );

        DebugMessagePrintf( "Disconnected from central MAC: %s\n", central.address().c_str() );
    }
}

void updateWrittings() {
    if ( readIMU( dataIMU ) ) {
        long currentTime = micros();
        lastInterval = currentTime - lastTime;
        lastTime = currentTime;

        doImuCalculations( dataIMU );
        printCalculations( dataIMU, lastInterval );
    }
}
