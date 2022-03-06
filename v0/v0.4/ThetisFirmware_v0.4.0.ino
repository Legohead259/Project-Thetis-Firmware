// General definitions
#include <Poseidon_Util.h>
#define BATT_MON_PIN 1
#define LOG_EN_PIN 12
#define ACT_LED_PIN 13
#define MCAL_LED_PIN 14
#define GCAL_LED_PIN 15
#define ACAL_LED_PIN 16

// GPS instantiation
#include <MicroNMEA.h>
#define GPS_PPS_PIN 42
#define GPS_FIX_PIN 43
HardwareSerial& GPS = Serial0;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool ledState = LOW;
volatile bool ppsTriggered = false;
uint8_t lastSecond = 0;
float lastMillis = millis();
float curMillis = 0;
float curMSecond = 0;

// BNO055 instantiation
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#define SAMPLE_RATE 32 // Hz
#define BNO_RST_PIN 5
#define BNO_SDA_PIN 26
#define BNO_SCL_PIN 33
Adafruit_BNO055 bno = Adafruit_BNO055(0x28); // Create BNO object with I2C addr 0x28

// XTSD instantiation
#include <SPI.h>
#include <SD.h>
#include "FS.h"
uint64_t cardSize;

// DEBUG flags
#define DEBUG_MODE false // Enable debugging to serial console - note, this will hang the code execution until serial port opened
#define GPSECHO false // Print GPS data verbose to serial port

void setup() {
    // Set pin modes
    pinMode(BATT_MON_PIN, INPUT);
    pinMode(LOG_EN_PIN, INPUT);
    pinMode(ACT_LED_PIN, OUTPUT);

    Serial.begin(115200);
    while(!Serial); // Wait for serial port to open

    initGPS();
    initIMU();
    initXTSD();
}

void loop() {
    
}

// ==============================
// ===INITIALIZATION FUNCTIONS===
// ==============================

void initGPS() {
    Serial.print("Initializing GPS..."); // DEBUG
    GPS.begin(9600); // Begin talking with GPS at 9600 baud
    if (!GPS) {
        Serial.println("Failed to initialize GPS"); // DEBUG
        while (true); // Block further code execution
            // TODO: Blink error code on activity LED
    }
    nmea.setUnknownSentenceHandler(printUnknownSentence); // Set interrupt Routine for unrecognized sentences
    MicroNMEA::sendSentence(GPS, "$PORZB"); // Clear the list of messages which are sent
    MicroNMEA::sendSentence(GPS, "$PORZB,RMC,1,GGA,1"); // Send only RMC (minimum recommended data) and GGA (fix data) including altitude
    MicroNMEA::sendSentence(GPS, "$PNVGNME,2,9,1"); // Disable compatability mode (NV08C-CSM proprietary message) and adjust precision of time and position fields

    pinMode(GPS_PPS_PIN, INPUT);
    pinMode(GPS_FIX_PIN, INPUT);
    attachInterrupt(GPS_PPS_PIN, ppsHandler, RISING);

    Serial.println("done!"); // DEBUG
}

void initIMU() {
    Serial.println("Initializing IMU..."); // DEBUG
    Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN); // Initialize I2C bus with correct wires
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055"); // DEBUG
        while (true); // Block further code execution
            // TODO: Blink error code on activity LED
    }
    bno.setExtCrystalUse(true);

    pinMode(MCAL_LED_PIN, OUTPUT);
    pinMode(GCAL_LED_PIN, OUTPUT);
    pinMode(ACAL_LED_PIN, OUTPUT);
    pinMode(BNO_RST_PIN, OUTPUT);

    Serial.println("done!"); // DEBUG
}

void initXTSD() {
    if (!SD.begin()) { // Check if SD card is present
        Serial.println("Card mount failed!");
        while(true); // Block further code execution
    }
    if (SD.cardType() == CARD_NONE) { // Check if SD card is of readable type
        Serial.println("No SD card attached");
        while(true); // Block further code execution
    }
    // TODO: Implement check for available storage space
}

// =======================
// ===POLLING FUNCTIONS===
// =======================

void pollGPS() {
    //Parse milliseconds for timestamp
    uint8_t curSecond = nmea.getSecond();
    // Serial.println(curSecond);
    if (curSecond == lastSecond) {
        curMillis = millis();
        curMSecond = curMillis - lastMillis;
        // Serial.println((int) curMSecond); //DEBUG
    }
    else {
        lastSecond = curSecond;
        lastMillis = millis();
        curMSecond = 0;
    }

    //Parse timestamp
    sprintf(data.timestamp, "%02d:%02d:%02d:%03d", nmea.getHour(), nmea.getMinute(), nmea.getSecond(), (int) curMSecond);

    //Update NMEA string based on PPS pulse from GPS. By default refresh rate is 1Hz
    if (ppsTriggered) {
        ppsTriggered = false;
        ledState = !ledState;
        digitalWrite(ACT_LED_PIN, ledState);

        data.GPSFix = nmea.isValid();
        data.numSats = nmea.getNumSatellites();
        data.HDOP = nmea.getHDOP();
        data.latitude = nmea.getLatitude();
        data.longitude = nmea.getLongitude();
        // nmea.getAltitude(data.altitudeMSL);
        data.GPSSpeed = nmea.getSpeed();
        data.GPSCourse = nmea.getCourse();
    }

    while (!ppsTriggered && GPS.available()) {
        char c = gps.read();
        nmea.process(c);
    }
}

void pollIMU() {
    bno.getCalibration(&data.sysCal, &data.gyroCal, &data.accelCal, &data.magCal);
    if (bno.isFullyCalibrated()) { //Don't read IMU data unless sensors are calibrated
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);    // - m/s^2
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         // - rad/s
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);            // - degrees
        imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // - m/s^2

        //Add accelerometer data to data packet            
        data.accelX = accel.x();
        data.accelY = accel.y();
        data.accelZ = accel.z();

        //Add gyroscope data to data packet
        data.gyroX = gyro.x();
        data.gyroY = gyro.y();
        data.gyroZ = gyro.z();
        
        //Add euler rotation data to data packet
        data.roll = euler.z();
        data.pitch = euler.y();
        data.yaw = euler.x();

        //Add linear accleration data to data packet
        data.linAccelX = linaccel.x();
        data.linAccelY = linaccel.y();
        data.linAccelZ = linaccel.z();
    }

    data.imuTemp = bno.getTemp();
}

// =======================
// ===UTILITY FUNCTIONS===
// =======================

void ppsHandler(void) {
	ppsTriggered = true;
	// Serial.println(\triggered!"); //DEBUG
}

void printUnknownSentence(const MicroNMEA& nmea) {
    // Needed for MicroNMEA library. Does not necessarily need to be used.
}

void updateStatusLEDs() {
    
}

// ====================
// ===XTSD FUNCTIONS===
// ====================

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

// =====================
// ===DEBUG FUNCTIONS===
// =====================