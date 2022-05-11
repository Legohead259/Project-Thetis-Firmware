// General definitions
#define LOG_EN_PIN 0
#define BATT_MON_PIN 1
#define ACT_LED_PIN 13
#define LOG_EN_HOLD_DOWN_TIME 2000 // ms
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
    FS_MOUNT_ERROR_CODE     = 0b0010,    // XTSD filesystem fails to mount
    CARD_TYPE_ERROR_CODE    = 0b0011,    // XTSD initializes, but reports a bad type
    FILE_ERROR_CODE         = 0b0100,    // Datalog file fails to open
    RADIO_ERROR_CODE        = 0b0101,    // ESP32 radio fails to initialize/encounters error
    GPS_ERROR_CODE          = 0b0110,    // GPS radio fails to initialize
    IMU_ERROR_CODE          = 0b0111,    // IMU fails to initialize
    LOW_BATT_ERROR_CODE     = 0b1000     // Battery voltage is below 3.0V
};

struct Telemetry {
    char timestamp[32];         // Timestamp in UTC obtained from GPS satellites
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
    float magZ;                 // mGausee
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
HardwareSerial GPS(0);
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
#define BNO_RST_PIN 5
#define BNO_SDA_PIN 26
#define BNO_SCL_PIN 33
#define SAMPLE_RATE 8 // Hz
volatile bool isIMUCalibrated = false; // IMU Calibration flag
Adafruit_BNO055 bno = Adafruit_BNO055(0x28); // Create BNO object with I2C addr 0x28

// XTSD instantiation
// #include <SPI.h>
// #include <SD.h>
#include <FS.h>
#include <FFat.h>
uint64_t cardSize;
char filename[12];

// Neopixel instantiation
#include <Adafruit_NeoPixel.h>
#define NEO_EN_PIN 14
#define NEO_DATA_PIN 15
#define DASH_ON 500
#define DOT_ON 125
#define BLINK_INTERVAL 250
#define MESSAGE_INTERVAL 1000
#define MAXIMUM_BRIGHTNESS 32
#define NUM_STEPS 16
#define BRIGHTNESS_STEP MAXIMUM_BRIGHTNESS/NUM_STEPS

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
#define DEBUG_MODE false // Enable debugging to serial console - note, this will hang the code execution until serial port opened
#define GPSECHO false   // Print GPS data verbose to serial port

void setup() {
    // Set pin modes
    pinMode(BATT_MON_PIN, INPUT);
    pinMode(LOG_EN_PIN, INPUT);
    pinMode(ACT_LED_PIN, OUTPUT);

    if (DEBUG_MODE) {
        Serial.begin(115200);
        while (!Serial); // Wait for serial port to open
    }

    attachInterrupt(LOG_EN_PIN, logEnableISR, FALLING);

    initNeoPixel();
    initGPS();
    initIMU();
    initFileSystem();

    if (DEBUG_MODE) {
        listDir(FFat, "/", 0);
    }
}

void loop() {
    // Check for log enable
    if (!digitalRead(LOG_EN_PIN)) isLogging = !isLogging; // If LOG_EN button is pressed, change the logging flag
    updateSystemState();
    updateSystemLED();
    data.voltage = analogReadMilliVolts(BATT_MON_PIN); // Update battery voltage
    pollIMU();
    pollGPS();
    // data.packetSize = sizeof(data);
    if (isLogging) writeTelemetryData();
    if (DEBUG_MODE) printTelemetryData();

    delay(1000/SAMPLE_RATE);
}

// ====================
// === IMU FUNCTIONS===
// ====================

void initIMU() {
    Serial.print("Initializing IMU...");
    Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN); // Initialize I2C bus with correct wires
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055");
        while (true) blinkCode(IMU_ERROR_CODE, RED);
    }
    bno.setExtCrystalUse(true);

    Serial.println("done!"); // DEBUG
}

void pollIMU() {
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


// =====================
// === GPS FUNCTIONS ===
// =====================


void initGPS() {
    Serial.print("Initializing GPS..."); // DEBUG
    GPS.begin(9600); // Begin talking with GPS at 9600 baud
    if (!GPS) {
        Serial.println("Failed to initialize GPS"); // DEBUG
        while (true); // Block further code execution
            // TODO: Blink error code on activity LED
    }
    // nmea.setUnknownSentenceHandler(printUnknownSentence); // Set interrupt Routine for unrecognized sentences
    MicroNMEA::sendSentence(GPS, "$PORZB"); // Clear the list of messages which are sent
    MicroNMEA::sendSentence(GPS, "$PORZB,RMC,1,GGA,1"); // Send only RMC (minimum recommended data) and GGA (fix data) including altitude
    MicroNMEA::sendSentence(GPS, "$PNVGNME,2,9,1"); // Disable compatability mode (NV08C-CSM proprietary message) and adjust precision of time and position fields

    pinMode(GPS_PPS_PIN, INPUT);
    // pinMode(GPS_FIX_PIN, INPUT);
    attachInterrupt(GPS_PPS_PIN, ppsHandler, RISING);

    Serial.println("done!"); // DEBUG
}

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
    sprintf(data.timestamp, "%04d-%02d-%02dT%02d:%02d:%02d.%03d", nmea.getYear(), nmea.getMonth(), nmea.getDay(), nmea.getHour(), nmea.getMinute(), nmea.getSecond(), (int) curMSecond);

    if (ppsTriggered) { //Update NMEA string based on PPS pulse from GPS. By default refresh rate is 1Hz
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
        char c = GPS.read();
        nmea.process(c);
    }
}

void ppsHandler(void) {
	ppsTriggered = true;
	// Serial.println(\triggered!"); //DEBUG
}

void printUnknownSentence(MicroNMEA& nmea) {
    Serial.println();
	Serial.print("Unknown sentence: ");
	Serial.println(nmea.getSentence());
}


// ======================
// === XTSD FUNCTIONS ===
// ======================


void initFileSystem() {
    Serial.print("Initializing storage system...");
    if (!FFat.begin()){
        Serial.println("Storage system initialization Failed");
        while(true) blinkCode(FS_MOUNT_ERROR_CODE, RED); // Block further code execution and flash error code
    }

    for (int i=0; i<255; i++) {
        sprintf(filename, "/Log_%03d.csv", i);
        if (!FFat.exists(filename)) 
            break;
    }
    Serial.printf("Logging to: %s\n\r", filename);
    
    // File _dataFile = FFat.open(filename, FILE_WRITE);
    // if (!_dataFile) {
    //     Serial.printf("Could not create %s", filename);
    //     while (true) blinkCode(FILE_ERROR_CODE, RED); // Block further code execution
    // }
    // // _dataFile.println("ISO 8601 Time,Battery Voltage (V),GPS Fix,# of Satellites,HDOP,Lat (deg),Lon (deg),Speed (kts),Course (kts),System Cal,Gyro Cal,Accel Cal,Mag Cal,Ax (m/s/s),Ay (m/s/s),Az (m/s/s),Gx (rad/s),Gy (rad/s),Gz (rad/s),Roll (deg),Pitch (deg),Yaw (deg),linAx (m/s/s),linAy (m/s/s),linAz (m/s/s),Qw,Qx,Qy,Qz,Temp (degC),State,Packet Size"); // Print header to file
    // _dataFile.println("ISO 8601 Time,Battery Voltage (V),GPS Fix,# of Satellites,HDOP,Lat (deg),Lon (deg),Speed (kts),Course (kts),System Cal,Gyro Cal,Accel Cal,Mag Cal,Ax (m/s/s),Ay (m/s/s),Az (m/s/s),Gx (rad/s),Gy (rad/s),Gz (rad/s),Roll (deg),Pitch (deg),Yaw (deg),linAx (m/s/s),linAy (m/s/s),linAz (m/s/s),State,Packet Size"); // Print header to file
    // _dataFile.close();
    writeFile(FFat, filename, "ISO 8601 Time,Battery Voltage (V),GPS Fix,# of Satellites,HDOP,Lat (deg),Lon (deg),System Cal,Gyro Cal,Accel Cal,Mag Cal,Ax (m/s/s),Ay (m/s/s),Az (m/s/s),Mx (mG),My (mG),Mz (mG),Roll (deg),Pitch (deg),Yaw (deg),State,\n\r");
}

void writeTelemetryData() {
    File _dataFile = FFat.open(filename, FILE_APPEND);
    if (!_dataFile) {
        Serial.printf("Could not create %s", filename);
        while (true) blinkCode(FILE_ERROR_CODE, RED); // Block further code execution
    }

    _dataFile.print(data.timestamp);
    _dataFile.printf("%0.3f,", data.voltage);
    _dataFile.printf("%d,", data.GPSFix);
    _dataFile.printf("%d,", data.numSats);
    _dataFile.printf("%d,", data.HDOP);
    _dataFile.printf("%0.3f,", data.latitude / 1E6);
    _dataFile.printf("%0.3f,", data.longitude / 1E6);
    // _dataFile.printf("%0.3f,", data.GPSSpeed / 1E3);
    // _dataFile.printf("%0.3f,", data.GPSCourse / 1E3);
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
    // _dataFile.printf("%0.3f,", data.gyroX);
    // _dataFile.printf("%0.3f,", data.gyroY);
    // _dataFile.printf("%0.3f,", data.gyroZ);
    _dataFile.printf("%0.3f,", data.roll);
    _dataFile.printf("%0.3f,", data.pitch);
    _dataFile.printf("%0.3f,", data.yaw);
    // _dataFile.printf("%0.3f,", data.linAccelX);
    // _dataFile.printf("%0.3f,", data.linAccelY);
    // _dataFile.printf("%0.3f,", data.linAccelZ);
    // _dataFile.printf("%0.3f,", data.quatW);
    // _dataFile.printf("%0.3f,", data.quatX);
    // _dataFile.printf("%0.3f,", data.quatY);
    // _dataFile.printf("%0.3f,", data.quatZ);
    // _dataFile.printf("%0.3f,", data.imuTemp);
    _dataFile.printf("%d,", data.state);
    // _dataFile.print(data.packetSize);
    _dataFile.println();
    _dataFile.close();

    Serial.printf("Wrote to: %s\n\r", filename);
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        while (true) blinkCode(FILE_ERROR_CODE, RED);
    }
    else {
        Serial.println("- opened file for writing");
    }

    if (file.println(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
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
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
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


void logEnableISR() {
    // static unsigned long last_interrupt_time = 0;
    // unsigned long interrupt_time = millis();
    // // If interrupts come faster than 100ms, assume it's a bounce and ignore
    // if (interrupt_time - last_interrupt_time > 100) {
    //     isLogging = !isLogging;
    //     Serial.printf("Logging %s\n\r", isLogging ? "enabled!" : "disabled!");
    // }
    // last_interrupt_time = interrupt_time;
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


static void getStateString(char* outStr, uint8_t s) {
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
    Serial.printf("Printing to:                 %s\n\r", filename);
    Serial.printf("Timestamp:                   %s\n\r", data.timestamp);
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
    Serial.printf("Storage free space:          %10uB", FFat.freeBytes());
    Serial.print("\n\n\r");
}