// General definitions
#define BATT_MON_PIN 1
#define LOG_EN_PIN 0
#define ACT_LED_PIN 13
#define MCAL_LED_PIN 14
#define GCAL_LED_PIN 15
#define ACAL_LED_PIN 16
#define TEST_TIME 10000 // 10 Seconds
int logEnablePresses = 0;

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

// DotStar instantiation
#define DASH_ON 250
#define DOT_ON 125
#define BLINK_INTERVAL 125
#define MESSAGE_INTERVAL 1000

Adafruit_DotStar strip(NUM_PIXELS, DOTSTAR_DATA_PIN, DOTSTAR_CLK_PIN, DOTSTAR_RGB);
const uint32_t OFF      =  strip.Color(0, 0, 0);       //BGR
const uint32_t WHITE    =  strip.Color(255, 255, 255);
const uint32_t BLUE     =  strip.Color(255, 0, 0);
const uint32_t RED      =  strip.Color(0, 0, 255);
const uint32_t GREEN    =  strip.Color(0, 255, 0);
const uint32_t PURPLE   =  strip.Color(255, 0, 255);
const uint32_t AMBER    =  strip.Color(0, 191, 255);
const uint32_t CYAN     =  strip.Color(255, 255, 0);
const uint32_t LIME     =  strip.Color(0, 255, 125);

// DEBUG flags
#define GPSECHO false // Print GPS data verbose to serial port

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for Serial port to open

    Serial.println("---------------------------------------");
    Serial.println("        Project Thetis Unit Test       ");
    Serial.println("                 REV F2                ");
    Serial.println("---------------------------------------");

    pinMode(ACT_LED_PIN, OUTPUT);
    pinMode(BATT_MON_PIN, INPUT);
    pinMode(LOG_EN_PIN, INPUT);
    attachInterrupt(LOG_EN_PIN, logEnableISR, FALLING);
    
    Serial.println("Testing LOG_EN button...");
    delay(TEST_TIME);
    Serial.println("done!");
    Serial.println("---------------------------------------");
    Serial.println();

    testBatteryMon();
    
    initGPS();
    testGPS();

    initIMU();
    testIMU();

    initXTSD();
    testXTSD();

    Serial.println("Unit test complete");
}

void loop() {

}

// ===================
// ===GPS FUNCTIONS===
// ===================

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

void testGPS() {
    Serial.println("Testing GPS...");
    long startTime = millis();
    while (millis() < startTime + TEST_TIME) { // For TEST_TIME, read GPS data from bus
        if (ppsTriggered) { // NOTE: NMEA parsing will not begin until 3D fix is acheived. See PPS information in datasheet for details
            ppsTriggered = false;
            ledState = !ledState;
            digitalWrite(ACT_LED_PIN, ledState);

            // Output GPS information from previous second
            Serial.print("Valid fix: ");
            Serial.println(nmea.isValid() ? "yes" : "no");

            Serial.print("Nav. system: ");
            if (nmea.getNavSystem())
                Serial.println(nmea.getNavSystem());
            else
                Serial.println("none");

            Serial.print("Num. satellites: ");
            Serial.println(nmea.getNumSatellites());

            Serial.print("HDOP: ");
            Serial.println(nmea.getHDOP()/10., 1);

            Serial.print("Date/time: ");
            Serial.print(nmea.getYear());
            Serial.print('-');
            Serial.print(int(nmea.getMonth()));
            Serial.print('-');
            Serial.print(int(nmea.getDay()));
            Serial.print('T');
            Serial.print(int(nmea.getHour()));
            Serial.print(':');
            Serial.print(int(nmea.getMinute()));
            Serial.print(':');
            Serial.println(int(nmea.getSecond()));

            long latitude_mdeg = nmea.getLatitude();
            long longitude_mdeg = nmea.getLongitude();
            Serial.print("Latitude (deg): ");
            Serial.println(latitude_mdeg / 1000000., 6);

            Serial.print("Longitude (deg): ");
            Serial.println(longitude_mdeg / 1000000., 6);

            long alt;
            Serial.print("Altitude (m): ");
            if (nmea.getAltitude(alt))
                Serial.println(alt / 1000., 3);
            else
                Serial.println("not available");

            Serial.print("Speed: ");
            Serial.println(nmea.getSpeed() / 1000., 3);
            Serial.print("Course: ");
            Serial.println(nmea.getCourse() / 1000., 3);
        }
        while (!ppsTriggered && GPS.available()) { // Print out raw GPS data when PPS is not triggered (no 3D fix)
            char c = GPS.read();
            Serial.print(c);
            nmea.process(c);
	    }
    }
    Serial.println("done!");
    Serial.println("---------------------------------------");
    Serial.println();
}

void ppsHandler(void) {
    ppsTriggered = true;
    // Serial.println("triggered!"); // DEBUG
}

void printUnknownSentence(MicroNMEA& nmea) {
    Serial.println();
	Serial.print("Unknown sentence: ");
	Serial.println(nmea.getSentence());
}

// ===================
// ===IMU FUNCTIONS===
// ===================

void initIMU() {
    Serial.print("Initializing IMU..."); // DEBUG
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

void testIMU() {
    Serial.println("Testing IMU...");
    long startTime = millis();
    while (millis() < startTime + TEST_TIME*6) { // For TEST_TIME, read off IMU data and update LEDs at SAMPLE_RATE
        // Possible vector values can be:
        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
        imu::Vector<3> accelerations = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

        /* Display the floating point data */
        Serial.print("X: ");
        Serial.print(accelerations.x());
        Serial.print(" Y: ");
        Serial.print(accelerations.y());
        Serial.print(" Z: ");
        Serial.print(accelerations.z());
        Serial.print("\t\t");

        // Quaternion data
        imu::Quaternion quat = bno.getQuat();
        Serial.print("qW: ");
        Serial.print(quat.w(), 4);
        Serial.print(" qX: ");
        Serial.print(quat.x(), 4);
        Serial.print(" qY: ");
        Serial.print(quat.y(), 4);
        Serial.print(" qZ: ");
        Serial.print(quat.z(), 4);
        Serial.print("\t\t");

        /* Display calibration status for each sensor. */
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.print("CALIBRATION: Sys=");
        Serial.print(system, DEC);
        Serial.print(" Gyro=");
        Serial.print(gyro, DEC);
        Serial.print(" Accel=");
        Serial.print(accel, DEC);
        Serial.print(" Mag=");
        Serial.println(mag, DEC);

        delay(100);
    }
    Serial.println("done!");
    Serial.println("---------------------------------------");
    Serial.println();
}

// ====================
// ===XTSD FUNCTIONS===
// ====================

void initXTSD() {
    Serial.print("Initializing XTSD card...");
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        while(true); // Block further code execution
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        while(true); // Block further code execution
    }
    Serial.println("done!");
}

void testXTSD() {
    Serial.println("Testing XTSD Card...");
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n\r", cardSize);

    listDir(SD, "/", 0);
    createDir(SD, "/mydir");
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n\r");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFileIO(SD, "/test.txt");
    Serial.printf("Total space: %lluMB\n\r", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n\r", SD.usedBytes() / (1024 * 1024));
    Serial.println("done!");
    Serial.println("---------------------------------------");
    Serial.println();
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n\r", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n\r", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n\r", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n\r", path);

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
    Serial.printf("Appending to file: %s\n\r", path);

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

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n\r", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n\r", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n\r", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n\r", 2048 * 512, end);
    file.close();
}

// =======================
// ===NEOPIXEL FUNCTIONS===
// =======================

void initNeoPixel() {
    strip.begin(); // Initialize pins for output
    strip.setBrightness(50);
    strip.show();  // Turn all LEDs off ASAP
}

// =======================
// ===GENERAL FUNCTIONS===
// =======================

void testBatteryMon() {
    Serial.println("Testing battery voltage monitoring...");
    long startTime = millis();
    while(millis() < startTime + TEST_TIME) { // For TEST_TIME, read the battery voltage every 500 ms
        Serial.printf("Battery voltage: %03f V\n\r", analogReadMilliVolts(BATT_MON_PIN)/1000);
        delay(500);
    }
    Serial.println("done!");
    Serial.println("---------------------------------------");
    Serial.println();
}

void logEnableISR() {
    logEnablePresses++;
    Serial.printf("Log button pressed: %d times\n\r", logEnablePresses);
}