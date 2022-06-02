#include <ThetisLib.h>

// BNO055 Instantiation
Adafruit_BNO055 BNO055_IMU = Adafruit_BNO055(0x28); // Create BNO object with I2C addr 0x28
bool isBNO055Available;

// LSM6DSO32 Instantiation
Adafruit_LSM6DSO32 DSO32_IMU;
bool isDSO32Available;

// Filesystem Instantiation

// GPS Instantiation
HardwareSerial& GPS = Serial1;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool ledState = LOW;
volatile bool ppsTriggered = false;

void setup() {
    Serial.begin(115200);
    while(!Serial); // wait for Serial port to open

    // IMU Initialization
    isBNO055Available = initBNO055(BNO055_IMU, Serial);
    isDSO32Available = initLSM6DSO32(DSO32_IMU, Serial);
    if (!isBNO055Available && !isDSO32Available) { // Check if both IMUs are unavailable
        while(true); // Block further code execution
    }

    // GPS Initialization
    if (!initGPS(GPS, Serial)) {
        while(true); // Block further code execution
    }

    // Filesystem Initialization
    if (!initFS(SD, Serial)) {
        while(true); // Block further code execution
    }
}

void loop() {
    
}