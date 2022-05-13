// General definitions
#define BATT_MON_PIN 15
#define ACT_LED_PIN 38
#define USB_DETECT_PIN 12
#define TEST_TIME 10000 // 10 Seconds
#define SAMPLE_RATE 8 // Hz

// Log enable definitions
#define LOG_EN_PIN 41
#define LOG_PRESS_TIME 1000 // ms
uint8_t logButtonPresses = 0;
long logButtonStartTime = 0;
bool isLogging = false;

/*
State Table:
State           |  Color  |  Indication  
----------------|---------|--------------
Error           |   RED   |   Pulsing     
Logging, No GPS |  BLUE   |    Solid      
Logging, GPS    |  GREEN  |    Solid      
Ready, No GPS   |  BLUE   |   Pulsing     
Ready, GPS      |  GREEN  |   Pulsing     
Standby         |  AMBER  |    Solid      
Booting         |  NONE   |     N/A       
*/

enum State {
    ERROR_STATE,            // Thetis has encountered some error
    LOGGING_NO_GPS,         // Thetis is logging, but does not have a GPS fix
    LOGGING_GPS,            // Thetis is logging with a GPS fix
    READY_NO_GPS,           // Accelerometer is calibrated but no GPS fix
    READY_GPS,              // Accelerometer is calibrated and there is a GPS fix
    STANDBY,                // Accelerometer is not calibrated yet
    BOOTING                 // Board is booting up
};
uint8_t currentState = BOOTING;

struct Telemetry {
    // NOTE: GPS timestamp information can be in time-since-start if GPS lock has not been acquired
    uint16_t year;              // Current year from GPS data
    uint8_t month;              // Current month from GPS data (1-12 inclusive)
    uint8_t day;                // Current day from GPS data (1-31 inclusive)
    uint8_t hour;               // Current hour from GPS data
    uint8_t minute;             // Current minute from GPS data
    uint8_t second;             // Current second from GPS data
    uint8_t millis;             // Current millisecond
    float voltage;              // Battery voltage in V
    bool GPSFix;                // If GPS has positive fix on location
    uint8_t numSats;            // Number of satellites GPS is communicating with
    uint8_t HDOP;               // Accuracy of GPS reading. Lower is better. In tenths (divide by 10. when displaying)
    long latitude;              // In millionths of a degree (divide by 1000000. when displaying)
    long longitude;             // In millionths of a degree (divide by 1000000. when displaying)
    long GPSSpeed;              // In thousandths of a knot (divide by 1000. when displaying)
    long GPSCourse;             // In thousandths of a degree (divide by 1000. when displaying)
    uint8_t sysCal = 0;         // IMU system calibration, 0-3 with 3 being fully calibrated
    uint8_t gyroCal = 0;        // IMU gyroscope calibration, 0-3 with 3 being fully calibrated
    uint8_t accelCal = 0;       // IMU accelerometer calibration, 0-3 with 3 being fully calibrated
    uint8_t magCal = 0;         // IMU magnetometer calibration, 0-3 with 3 being fully calibrated
    float accelX;               // m/s^2
    float accelY;               // m/s^2
    float accelZ;               // m/s^2
    float gyroX;                // rad/s
    float gyroY;                // rad/s
    float gyroZ;                // rad/s
    float magX;                 // mGauss
    float magY;                 // mGauss
    float magZ;                 // mGauss
    float roll;                 // degrees
    float pitch;                // degrees
    float yaw;                  // degrees
    float linAccelX;            // m/s^2
    float linAccelY;            // m/s^2
    float linAccelZ;            // m/s^2
    float quatW;                //
    float quatX;                //
    float quatY;                //
    float quatZ;                //
    float imuTemp;              // °Celsius from the IMU
    uint8_t state;              // State reported by the package.
    uint8_t packetSize;         // The size of the telemetry packet. Used as a debug tool for ground station/thetis comms.
};
Telemetry data;

// GPS instantiation
#include <MicroNMEA.h>
#define GPS_PPS_PIN 42
HardwareSerial& GPS = Serial1;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// BNO055 instantiation
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#define BNO_RST_PIN 7
#define SDA_PIN 33 
#define SCL_PIN 34
bool isIMUCalibrated = false; // IMU Calibration flag
Adafruit_BNO055 bno = Adafruit_BNO055(0x28); // Create BNO object with I2C addr 0x28
bool isBNOAvailable = false; // IMU initialization flag

// LSM6DSO instantiation
#include <Adafruit_LSM6DSO32.h>
Adafruit_LSM6DSO32 DSO32_IMU;
bool isDSO32Available; // IMU initialization flag

// Storage instantiation
#include "FS.h"
#include "SD.h"
#include "SPI.h"
char filename[12];

// Neopixel instantiation
#include <Adafruit_NeoPixel.h>
#define NEO_EN_PIN 39
#define NEO_DATA_PIN 40
#define DASH_ON 500
#define DOT_ON 125
#define BLINK_INTERVAL 250
#define MESSAGE_INTERVAL 1000
#define MAXIMUM_BRIGHTNESS 32
#define NUM_STEPS 16
#define BRIGHTNESS_STEP MAXIMUM_BRIGHTNESS/NUM_STEPS

/*
Code Table:
Error       |  DOT  |  DASH  |  DOT  |  DASH  |  CODE  |  COLOR  
------------|-------|--------|-------|--------|--------|---------
General     |   0   |   0    |   0   |    1   |  B0001 |   RED    
XTSD Mount  |   0   |   0    |   1   |    0   |  B0010 |   RED    
Card Type   |   0   |   0    |   1   |    1   |  B0011 |   RED    
File Write  |   0   |   1    |   0   |    0   |  B0100 |   RED    
Radio       |   0   |   1    |   0   |    1   |  B0101 |   RED    
GPS         |   0   |   1    |   1   |    0   |  B0110 |   RED    
IMU         |   0   |   1    |   1   |    1   |  B0111 |   RED    
Low Battery |   1   |   0    |   0   |    0   |  B1000 |  AMBER   

A dot is 125 ms on, 125 ms off
A dash is 500 ms on, 500 ms off
Space between codes is 1 sec
*/

enum ErrorCode {
    GEN_ERROR_CODE          = 0b0001,    // Unknown, but critical failure
    FS_MOUNT_ERROR_CODE     = 0b0010,    // Filesystem fails to mount
    CARD_TYPE_ERROR_CODE    = 0b0011,    // Filesystem initializes, but reports a bad type
    FILE_ERROR_CODE         = 0b0100,    // Datalog file fails to open
    RADIO_ERROR_CODE        = 0b0101,    // ESP32 radio fails to initialize/encounters error
    GPS_ERROR_CODE          = 0b0110,    // GPS radio fails to initialize
    IMU_ERROR_CODE          = 0b0111,    // IMU fails to initialize
    LOW_BATT_ERROR_CODE     = 0b1000     // Battery voltage is below 3.0V
};

Adafruit_NeoPixel pixel(1, NEO_DATA_PIN, NEO_RGB + NEO_KHZ800);
const uint32_t OFF      =  pixel.Color(0, 0, 0);
const uint32_t WHITE    =  pixel.Color(255, 255, 255);
const uint32_t BLUE     =  pixel.Color(255, 0, 0);
const uint32_t RED      =  pixel.Color(0, 255, 0);
const uint32_t GREEN    =  pixel.Color(0, 255, 0);
const uint32_t PURPLE   =  pixel.Color(255, 0, 255);
const uint32_t AMBER    =  pixel.Color(255, 191, 0);
const uint32_t CYAN     =  pixel.Color(255, 255, 0);
const uint32_t LIME     =  pixel.Color(0, 255, 125);
const float brightness = 0.1;
uint8_t pixelState = 0;
bool brightnessInc = true;

// DEBUG flags
#define GPSECHO false   // Print GPS data verbose to serial port
bool DEBUG_MODE = false;

void setup() {
    // Set pin modes
    pinMode(BATT_MON_PIN, INPUT);
    pinMode(LOG_EN_PIN, INPUT);
    pinMode(ACT_LED_PIN, OUTPUT);
    pinMode(USB_DETECT_PIN, INPUT);

    DEBUG_MODE = digitalRead(USB_DETECT_PIN); 
    if (DEBUG_MODE) {
        Serial.begin(115200);
        while (!Serial); // Wait for serial port to open
    }

    attachInterrupt(LOG_EN_PIN, logEnableISR, FALLING);

    initNeoPixel();
    initGPS();
    initBNO055();
    initDSO32();
    initFileSystem();
}

void loop() {
    // Check for log enable
    static uint8_t _oldLogButtonPresses = logButtonPresses;

    if (logButtonPresses != _oldLogButtonPresses && !digitalRead(LOG_EN_PIN) && millis() >= logButtonStartTime+LOG_PRESS_TIME) { // Check if log button is pressed and has been held
        isLogging = !isLogging;
        _oldLogButtonPresses = logButtonPresses;
        Serial.printf("Logging is %s!\r\n", isLogging ? "enabled" : "disabled"); // DEBUG
    }

    updateSystemState();
    updateSystemLED();
    data.voltage = analogReadMilliVolts(BATT_MON_PIN); // Update battery voltage
    if (isBNOAvailable) pollBNO055();
    if (isDSO32Available && !isBNOAvailable) pollDSO32();
    if (isLogging) writeTelemetryData();
    if (DEBUG_MODE) printTelemetryData();
    data.packetSize = sizeof(data);

    delay(1000/SAMPLE_RATE);
}

// ====================
// === IMU FUNCTIONS===
// ====================

bool initBNO055() {
    Serial.print("Initializing IMU...");
    Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C bus with correct wires
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055");
        isBNOAvailable = false;
        isIMUCalibrated = true;
    }
    else {
        bno.setExtCrystalUse(true);
        isBNOAvailable = true;
        Serial.println("done!"); // DEBUG
    }
    return isBNOAvailable;
}

void pollBNO055() {
    bno.getCalibration(&data.sysCal, &data.gyroCal, &data.accelCal, &data.magCal);
    isIMUCalibrated = data.gyroCal == 3 && data.accelCal == 3 && data.magCal == 3; // Check if IMU has full calibration
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);    // - m/s^2
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         // - rad/s
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);            // - degrees
    imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // - m/s^2
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);       // - mGauss

    // Add accelerometer data to data packet            
    data.accelX = accel.x();
    data.accelY = accel.y();
    data.accelZ = accel.z();

    // Add gyroscope data to data packet
    data.gyroX = gyro.x();
    data.gyroY = gyro.y();
    data.gyroZ = gyro.z();
    
    // Add euler rotation data to data packet
    data.roll = euler.z();
    data.pitch = euler.y();
    data.yaw = euler.x();

    // Add magnetometer data to data packet
    data.magX = mag.x();
    data.magY = mag.y();
    data.magZ = mag.z();

    // Add linear accleration data to data packet
    data.linAccelX = linaccel.x();
    data.linAccelY = linaccel.y();
    data.linAccelZ = linaccel.z();

    // Add Quaternion data to packet
    imu::Quaternion quat = bno.getQuat();
    data.quatW = quat.w();
    data.quatX = quat.x();
    data.quatY = quat.y();
    data.quatZ = quat.z();

    data.imuTemp = bno.getTemp();
}

bool initDSO32() {
    Serial.print("Initializing DSO32 IMU..."); // DEBUG
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!DSO32_IMU.begin_I2C(0x6B)) {
        Serial.println("Failed to find LSM6DSO32 chip"); // DEBUG
        isDSO32Available = false;
    }
    else {
        DSO32_IMU.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G); // Set acceleration range to ±8g
        DSO32_IMU.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS); // Set gyroscope range to ±2000 deg/sec 
        DSO32_IMU.setAccelDataRate(LSM6DS_RATE_52_HZ);      // Set accelerometer update rate to 52 Hz
        DSO32_IMU.setGyroDataRate(LSM6DS_RATE_52_HZ);       // Set gyroscope update rate to 52 Hz
        Serial.println("done!"); // DEBUG
        isDSO32Available = true;
    }
    return isDSO32Available;
}

void pollDSO32() {
    DSO32_IMU.readAcceleration(data.accelX, data.accelY, data.accelZ);  // Read acceleration values in and save them to the data packet
    DSO32_IMU.readGyroscope(data.gyroX, data.gyroY, data.gyroZ);        // Read gyroscope values in and save them to the data packet
    data.imuTemp = DSO32_IMU.temperature;
    // TODO: process data into Roll/Pitch/Yaw
    // TODO: process data into linear acceleleration values
    // TODO: process data into quaternion data
}


// =====================
// === GPS FUNCTIONS ===
// =====================


void initGPS() {
    Serial.print("Initializing GPS..."); // DEBUG
    GPS.begin(9600); // Begin talking with GPS at default 9600 baud.
    // TODO: Automatically determine GPS initial baudrate
    if (!GPS) {
        Serial.println("Failed to initialize GPS"); // DEBUG
        while (true) // Block further code execution
            blinkCode(GPS_ERROR_CODE, RED);
    }
    MicroNMEA::sendSentence(GPS, "$PMTK251,38400");         // Set GPS baudrate to 38400
    GPS.begin(38400);                                       // Synchronize Thetis baudrate to GPS
    MicroNMEA::sendSentence(GPS, "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0"); // Enable only NMEA GGA sentences
    MicroNMEA::sendSentence(GPS, "$PMTK220,100");           // Set GPS update rate to 100 ms (10 Hz)

    nmea.setUnknownSentenceHandler(printUnknownSentence);   // Set interrupt routine for unrecognized sentences

    Serial.println("done!"); // DEBUG
}

void pollGPS() {
    static bool _newGPSData = false;
    while (GPS.available()) { // Read in the GPS string and parse it
        _newGPSData = true;
        char c = GPS.read();
        nmea.process(c);
        Serial.print(c); // DEBUG
    }

    // Parse milliseconds for timestamp
    static long _lastMillis = 0;
    long _curMillis = millis();
    static uint8_t _lastSecond = 0;
    uint8_t _curSecond = data.second;

    // Serial.println(curSecond);
    if (_curSecond == _lastSecond) {
        data.millis = _curMillis - _lastMillis;
        // Serial.println((int) curMSecond); //DEBUG
    }
    else {
        _lastSecond = _curSecond;
        _lastMillis = millis();
        _curMillis = 0;
    }

    if (_newGPSData) {
        data.year = nmea.getYear();
        data.month = nmea.getMonth();
        data.day = nmea.getDay();
        data.hour = nmea.getHour();
        data.minute = nmea.getMinute();
        data.second = nmea.getSecond();

        data.GPSFix = nmea.isValid();
        data.numSats = nmea.getNumSatellites();
        data.HDOP = nmea.getHDOP();
        data.latitude = nmea.getLatitude();
        data.longitude = nmea.getLongitude();
        data.GPSSpeed = nmea.getSpeed();
        data.GPSCourse = nmea.getCourse();

        _newGPSData = false;
    }
}

void printUnknownSentence(MicroNMEA& nmea) {
    Serial.println();
	Serial.print("Unknown sentence: ");
	Serial.println(nmea.getSentence());
}


// ============================
// === FILESYSTEM FUNCTIONS ===
// ============================


void initFileSystem() {
    Serial.print("Initializing filesystem...");
    if (!SD.begin()) {
        Serial.println("Card Mount Failed");
        while(true) blinkCode(FS_MOUNT_ERROR_CODE, RED); // Block further code execution and flash error code
    }

    uint8_t cardType = SD.cardType();
    if(cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        while(true) blinkCode(CARD_TYPE_ERROR_CODE, RED); // Block further code execution and flash error code
    }
    Serial.println("done!");

    for (int i=0; i<255; i++) { // Determine  the log file name
        sprintf(filename, "/Log_%03d.csv", i);
        if (!SD.exists(filename)) break; // Find an unused log file name and stop trying to come up with a new one
    }
    Serial.printf("Logging to: %s\n\r", filename); // DEBUG
    
    // Write headers to log file
    File _dataFile = SD.open(filename, FILE_WRITE);
    if (!_dataFile) {
        Serial.printf("Could not create %s", filename);
        while (true) blinkCode(FILE_ERROR_CODE, RED); // Block further code execution
    }
    _dataFile.println("ISO 8601 Time,Battery Voltage (V),GPS Fix,# of Satellites,HDOP,Lat (deg),Lon (deg),Speed (kts),Course (kts),System Cal,Gyro Cal,Accel Cal,Mag Cal,Ax (m/s/s),Ay (m/s/s),Az (m/s/s),Gx (rad/s),Gy (rad/s),Gz (rad/s),Roll (deg),Pitch (deg),Yaw (deg),linAx (m/s/s),linAy (m/s/s),linAz (m/s/s),Qw,Qx,Qy,Qz,Temp (degC),State,Packet Size"); // Print header to file
    _dataFile.close();
}

void writeTelemetryData() {
    File _dataFile = SD.open(filename, FILE_APPEND);
    if (!_dataFile) {
        Serial.printf("Could not create %s", filename);
        while (true) blinkCode(FILE_ERROR_CODE, RED); // Block further code execution
    }

    char _timestamp[32]; sprintf(_timestamp, "%04d-%02d-%02dT%02d:%02d:%02d.%03d", data.year, data.month, data.day, data.hour, data.minute, data.second, data.millis);
    _dataFile.print(_timestamp);
    _dataFile.printf("%0.3f,", data.voltage);
    _dataFile.printf("%d,", data.GPSFix);
    _dataFile.printf("%d,", data.numSats);
    _dataFile.printf("%d,", data.HDOP);
    _dataFile.printf("%0.3f,", data.latitude / 1E6);
    _dataFile.printf("%0.3f,", data.longitude / 1E6);
    _dataFile.printf("%0.3f,", data.GPSSpeed / 1E3);
    _dataFile.printf("%0.3f,", data.GPSCourse / 1E3);
    _dataFile.printf("%d,", data.sysCal);
    _dataFile.printf("%d,", data.gyroCal);
    _dataFile.printf("%d,", data.accelCal);
    _dataFile.printf("%d,", data.magCal);
    _dataFile.printf("%0.3f,", data.accelX);
    _dataFile.printf("%0.3f,", data.accelY);
    _dataFile.printf("%0.3f,", data.accelZ);
    _dataFile.printf("%0.3f,", data.magX);
    _dataFile.printf("%0.3f,", data.magY);
    _dataFile.printf("%0.3f,", data.magZ);
    _dataFile.printf("%0.3f,", data.gyroX);
    _dataFile.printf("%0.3f,", data.gyroY);
    _dataFile.printf("%0.3f,", data.gyroZ);
    _dataFile.printf("%0.3f,", data.roll);
    _dataFile.printf("%0.3f,", data.pitch);
    _dataFile.printf("%0.3f,", data.yaw);
    _dataFile.printf("%0.3f,", data.linAccelX);
    _dataFile.printf("%0.3f,", data.linAccelY);
    _dataFile.printf("%0.3f,", data.linAccelZ);
    _dataFile.printf("%0.3f,", data.quatW);
    _dataFile.printf("%0.3f,", data.quatX);
    _dataFile.printf("%0.3f,", data.quatY);
    _dataFile.printf("%0.3f,", data.quatZ);
    _dataFile.printf("%0.3f,", data.imuTemp);
    _dataFile.printf("%d,", data.state);
    _dataFile.print(data.packetSize);
    _dataFile.println();
    _dataFile.close();

    Serial.printf("Wrote to: %s\n\r", filename); // DEBUG
}


// ========================
// ===NEOPIXEL FUNCTIONS===
// ========================


void initNeoPixel() {
    Serial.print("Initializing NeoPixel...");
    pinMode(NEO_EN_PIN, OUTPUT);
    digitalWrite(NEO_EN_PIN, LOW); // Enable NeoPixel
    pixel.begin(); // Initialize pins for output
    pixel.setBrightness(25);
    pixel.show();  // Turn all LEDs off ASAP
    Serial.println("done!");
}

void pulseLED(uint32_t color) {
    pixel.setBrightness(pixelState);
    pixel.setPixelColor(0, color);
    pixel.show();
    brightnessInc ? pixelState += BRIGHTNESS_STEP : pixelState -= BRIGHTNESS_STEP;
    if (pixelState >= MAXIMUM_BRIGHTNESS || pixelState <= 0) brightnessInc = !brightnessInc;
}

void rainbow(){
    pixel.setPixelColor(0, Wheel(&pixel,pixelState));
    pixel.show();
    pixelState++;
    if (pixelState >= 255) pixelState = 0;
}

uint32_t Wheel(Adafruit_NeoPixel *strip, byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
        return strip->Color((255 - WheelPos * 3)*brightness, 0, WheelPos * 3*brightness);
    }
    if(WheelPos < 170) {
        WheelPos -= 85;
        return strip->Color(0, WheelPos * 3 * brightness, (255 - WheelPos * 3)*brightness);
    }
    WheelPos -= 170;
    return strip->Color(WheelPos * 3 * brightness, (255 - WheelPos * 3)*brightness, 0);
}

void blinkCode(byte code, uint32_t color) {
    bool dash = true;
    for (int n=0; n<4; n++) {
        if (bitRead(code, n)) {
            if (dash) {
                pixel.setPixelColor(0, color); pixel.show();
                delay(DASH_ON);
                pixel.setPixelColor(0, OFF); pixel.show();
                delay(BLINK_INTERVAL);
            }
            else {
                pixel.setPixelColor(0, color); pixel.show();
                delay(DOT_ON);
                pixel.setPixelColor(0, OFF); pixel.show();
                delay(BLINK_INTERVAL);
            }
        }
        else {
            if (dash) delay(DASH_ON+BLINK_INTERVAL);
            else delay(DOT_ON+BLINK_INTERVAL);
        }
        dash = !dash;
    }
    delay(MESSAGE_INTERVAL);
}


// =======================
// ===GENERAL FUNCTIONS===
// =======================


void checkBatteryVoltage() {
    analogSetAttenuation(ADC_0db);
    double rawADCVal = analogRead(BATT_MON_PIN);
    double voltagePerNum = 1.03/8192.0; // 1.1 = Vref
    double vBatMeasured = rawADCVal * voltagePerNum;
    double vBat = (vBatMeasured * (1E6 + 1E6)) / 1.0E6 * 2;
    data.voltage = vBat;
}

void logEnableISR() {
    logButtonPresses++;
    logButtonStartTime = millis();
    Serial.printf("Log button pressed: %d times\n\r", logButtonPresses); // DEBUG
}

void updateSystemState() {
    if (!isIMUCalibrated && !data.GPSFix)                       currentState = STANDBY;
    else if (isIMUCalibrated && !data.GPSFix && !isLogging)     currentState = READY_NO_GPS;
    else if (isIMUCalibrated && data.GPSFix && !isLogging)      currentState = READY_GPS;
    else if (isIMUCalibrated && !data.GPSFix && isLogging)      currentState = LOGGING_NO_GPS;
    else if (isIMUCalibrated && data.GPSFix && isLogging)       currentState = LOGGING_GPS;
    data.state = currentState;
}

void updateSystemLED() {
    switch (data.state) {
        case ERROR_STATE:
            // Inidividual errors will update the LED themselves
            break;
        case LOGGING_NO_GPS:
            pixel.setPixelColor(0, BLUE); pixel.show(); // Glow solid blue
            break;
        case LOGGING_GPS:
            pixel.setPixelColor(0, GREEN); pixel.show(); // Glow solid green
            break;
        case READY_NO_GPS:
            pulseLED(BLUE); // Pulse blue
            break;
        case READY_GPS:
            pulseLED(GREEN); // Pulse green
            break;
        case STANDBY:
            pixel.setPixelColor(0, 255, 191, 0); pixel.show(); // Glow solid amber
            break;
        case BOOTING:
            pulseLED(PURPLE); // Pulse purple
            break;
        default:
            pixel.setPixelColor(0, RED); pixel.show(); // Turn off LED
            break;
    }
}


// =====================
// ===DEBUG FUNCTIONS===
// =====================


void getStateString(char* outStr, uint8_t s) {
    switch(s) {
        case ERROR_STATE:
            strcpy(outStr, "ERROR");
            break;
        case LOGGING_NO_GPS:
            strcpy(outStr, "LOGGING_NO_GPS");
            break;
        case LOGGING_GPS:
            strcpy(outStr, "LOGGING_GPS");
            break;
        case READY_NO_GPS:
            strcpy(outStr, "READY_NO_GPS");
            break;
        case READY_GPS:
            strcpy(outStr, "READY_GPS");
            break;
        case STANDBY:
            strcpy(outStr, "STANDBY");
            break;
        case BOOTING:
            strcpy(outStr, "BOOTING");
            break;
        default:
            strcpy(outStr, "DINGUS");
            break;
    }
}

void printTelemetryData() {
    char _statestr[16]; getStateString(_statestr, data.state);
    char _timestamp[32]; sprintf(_timestamp, "%04d-%02d-%02dT%02d:%02d:%02d.%03d", data.year, data.month, data.day, data.hour, data.minute, data.second, data.millis);
    Serial.printf("Printing to:                 %s\n\r", filename);
    Serial.printf("Timestamp:                   %s\n\r", _timestamp);
    Serial.printf("Battery Voltage:             %0.3f V\n\r", data.voltage);
    Serial.printf("GPS Fix:                     %s\n\r", data.GPSFix ? "true" : "false");
    Serial.printf("Number of Satellites:        %d\n\r", data.numSats);
    Serial.printf("HDOP:                        %d\n\r", data.HDOP);
    Serial.printf("Latitude:                    %0.6f°\n\r", data.longitude/1E6);
    Serial.printf("Longitude:                   %0.6f°\n\r", data.latitude/1E6);
    Serial.printf("GPS Speed:                   %0.3f kts\n\r", data.GPSSpeed/1E3);
    Serial.printf("GPS Course:                  %0.3f°\n\r", data.GPSCourse);
    Serial.printf("System Calibration:          %d\n\r", data.sysCal);
    Serial.printf("Gyroscope Calibration:       %d\n\r", data.gyroCal);
    Serial.printf("Accelerometer Calibration:   %d\n\r", data.accelCal);
    Serial.printf("Magnetometer Calibration:    %d\n\r", data.magCal);
    Serial.printf("Acceleration X:              %0.3f m/s/s\n\r", data.accelX);
    Serial.printf("             Y:              %0.3f m/s/s\n\r", data.accelY);
    Serial.printf("             Z:              %0.3f m/s/s\n\r", data.accelZ);
    Serial.printf("Gyroscope X:                 %0.3f rad/s\n\r", data.gyroX);
    Serial.printf("          Y:                 %0.3f rad/s\n\r", data.gyroY);
    Serial.printf("          Z:                 %0.3f rad/s\n\r", data.gyroZ);
    Serial.printf("Magnetometer X:              %0.3f mGauss\n\r", data.magX);
    Serial.printf("Magnetometer Y:              %0.3f mGauss\n\r", data.magY);
    Serial.printf("Magnetometer Z:              %0.3f mGauss\n\r", data.magZ);
    Serial.printf("Roll:                        %0.3f°\n\r", data.roll);
    Serial.printf("Pitch:                       %0.3f°\n\r", data.pitch);
    Serial.printf("Yaw:                         %0.3f°\n\r", data.yaw);
    Serial.printf("Linear Acceleration X:       %0.3f m/s/s\n\r", data.linAccelX);
    Serial.printf("                    Y:       %0.3f m/s/s\n\r", data.linAccelY);
    Serial.printf("                    Z:       %0.3f m/s/s\n\r", data.linAccelZ);
    Serial.printf("Quaternion W:                %0.3f\n\r", data.quatW);
    Serial.printf("           X:                %0.3f\n\r", data.quatX);
    Serial.printf("           Y:                %0.3f\n\r", data.quatY);
    Serial.printf("           Z:                %0.3f\n\r", data.quatZ);
    Serial.printf("IMU Temperature:             %0.3f°C\n\r", data.imuTemp);
    Serial.printf("Thetis State:                %s\n\r", _statestr);
    Serial.printf("Packet Size:                 %d\n\r", data.packetSize);
    Serial.printf("Storage used space:          %10uB", SD.usedBytes());
    Serial.print("\n\n\r");
}