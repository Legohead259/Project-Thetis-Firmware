// General definitions
#define LOG_EN_PIN 0
#define BATT_MON_PIN 1
#define ACT_LED_PIN 13
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
    ERROR_STATE = -1,       // Thetis has encountered some error
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
    XTSD_MOUNT_ERROR_CODE   = 0b0010,    // XTSD filesystem fails to mount
    CARD_TYPE_ERROR_CODE    = 0b0011,    // XTSD initializes, but reports a bad type
    FILE_ERROR_CODE         = 0b0100,    // Datalog file fails to open
    RADIO_ERROR_CODE        = 0b0101,    // ESP32 radio fails to initialize/encounters error
    GPS_ERROR_CODE          = 0b0110,    // GPS radio fails to initialize
    IMU_ERROR_CODE          = 0b0111,    // IMU fails to initialize
    LOW_BATT_ERROR_CODE     = 0b1000     // Battery voltage is below 3.0V
};

struct Telemetry {
    float voltage;              // Battery voltage in V
    uint8_t month;              // Month from GPS data 
    uint8_t day;                // Day from GPS data
    uint16_t year;              // Year from GPS data
    char timestamp[32];         // Timestamp in UTC obtained from GPS satellites
    bool GPSFix;                // If GPS has positive fix on location
    // uint8_t numSats;            // Number of satellites GPS is communicating with
    uint8_t HDOP;               // Accuracy of GPS reading. Lower is better. In tenths (divide by 10. when displaying)
    long latitude;              // In millionths of a degree (divide by 1000000. when displaying)
    long longitude;             // In millionths of a degree (divide by 1000000. when displaying)
    long GPSSpeed;              // In thousandths of a knot (divide by 1000. when displaying)
    long GPSCourse;             // In thousandths of a degree (divide by 1000. when displaying)
    uint8_t sysCal = 0;         // IMU system calibration, 0-3 with 3 being fully calibrated
    uint8_t gyroCal = 0;        // IMU gyroscope calibration, 0-3 with 3 being fully calibrated
    uint8_t accelCal = 0;       // IMU accelerometer calibration, 0-3 with 3 being fully calibrated
    uint8_t magCal = 0;         // IMU magnetometer calibration, 0-3 with 3 being fully calibrated
    // float accelX;               // m/s^2
    // float accelY;               // m/s^2
    // float accelZ;               // m/s^2
    // float gyroX;                // rad/s
    // float gyroY;                // rad/s
    // float gyroZ;                // rad/s
    float roll;                 // degrees
    float pitch;                // degrees
    float yaw;                  // degrees
    float linAccelX;            // m/s^2
    float linAccelY;            // m/s^2
    float linAccelZ;            // m/s^2
    // float quatW;                //
    // float quatX;                //
    // float quatY;                //
    // float quatZ;                //
    // float imuTemp;              // °Celsius from the IMU
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
#define SAMPLE_RATE 32 // Hz
volatile bool isIMUCalibrated = false; // IMU Calibration flag
Adafruit_BNO055 bno = Adafruit_BNO055(0x28); // Create BNO object with I2C addr 0x28

// XTSD instantiation
#include <SPI.h>
#include <SD.h>
#include <FS.h>
uint64_t cardSize;

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

Adafruit_NeoPixel pixel(1, NEO_DATA_PIN, NEO_GRB + NEO_KHZ800);
const uint32_t OFF      =  pixel.Color(0, 0, 0);       // GRB?
const uint32_t WHITE    =  pixel.Color(255, 255, 255);
const uint32_t BLUE     =  pixel.Color(255, 0, 0);
const uint32_t RED      =  pixel.Color(0, 255, 0);
const uint32_t GREEN    =  pixel.Color(0, 255, 0);
const uint32_t PURPLE   =  pixel.Color(255, 0, 255);
const uint32_t AMBER    =  pixel.Color(0, 255, 0);
const uint32_t CYAN     =  pixel.Color(255, 255, 0);
const uint32_t LIME     =  pixel.Color(0, 255, 125);
const float brightness = 0.1;
uint8_t pixelState = 0;
bool brightnessInc = true;

// DEBUG flags
#define DEBUG_MODE true // Enable debugging to serial console - note, this will hang the code execution until serial port opened
#define GPSECHO false // Print GPS data verbose to serial port

void setup() {
    // Set pin modes
    pinMode(BATT_MON_PIN, INPUT);
    pinMode(LOG_EN_PIN, INPUT);
    pinMode(ACT_LED_PIN, OUTPUT);

    if (DEBUG_MODE) {
        Serial.begin(115200);
        while (!Serial); // Wait for serial port to open
    }

    initNeoPixel();
    initGPS();
    initIMU();
    initXTSD();
}

void loop() {
    updateSystemState();
    updateSystemLED();
    data.voltage = analogReadMilliVolts(BATT_MON_PIN) / 1000.0; // Update battery voltage
    pollIMU();
    pollGPS();
    data.state = currentState;
    data.packetSize = sizeof(data);

    printTelemetryData();
    delay(1250);
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
    isIMUCalibrated = bno.isFullyCalibrated();
    if (isIMUCalibrated) { // Don't read IMU data unless sensors are calibrated
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);    // - m/s^2
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         // - rad/s
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);            // - degrees
        imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // - m/s^2

        //Add accelerometer data to data packet            
        // data.accelX = accel.x();
        // data.accelY = accel.y();
        // data.accelZ = accel.z();

        //Add gyroscope data to data packet
        // data.gyroX = gyro.x();
        // data.gyroY = gyro.y();
        // data.gyroZ = gyro.z();
        
        //Add euler rotation data to data packet
        data.roll = euler.z();
        data.pitch = euler.y();
        data.yaw = euler.x();

        //Add linear accleration data to data packet
        data.linAccelX = linaccel.x();
        data.linAccelY = linaccel.y();
        data.linAccelZ = linaccel.z();

        // Add Quaternion data to packet
        // imu::Quaternion quat = bno.getQuat();
        // data.quatW = quat.w();
        // data.quatX = quat.x();
        // data.quatY = quat.y();
        // data.quatZ = quat.z();
    }

    // data.imuTemp = bno.getTemp();
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
    sprintf(data.timestamp, "%02d:%02d:%02d:%03d", nmea.getHour(), nmea.getMinute(), nmea.getSecond(), (int) curMSecond);

    if (ppsTriggered) { //Update NMEA string based on PPS pulse from GPS. By default refresh rate is 1Hz
        ppsTriggered = false;
        ledState = !ledState;
        digitalWrite(ACT_LED_PIN, ledState);

        data.GPSFix = nmea.isValid();
        // data.numSats = nmea.getNumSatellites();
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


void initXTSD() {
    Serial.print("Initializing XTSD card...");
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        while(true) blinkCode(XTSD_MOUNT_ERROR_CODE, RED); // Block further code execution and flash error code
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        while(true) blinkCode(CARD_TYPE_ERROR_CODE, RED); // Block further code execution and flash error code
    }
    Serial.println("done!");
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
    // TODO: Implement software button debounce
    isLogging = !isLogging;
    isLogging ? Serial.printf("Logging enabled!\n\r") : Serial.printf("Logging disabled!\n\r");
}

void updateSystemState() {
    if (!isIMUCalibrated && !data.GPSFix && !isLogging)         currentState = STANDBY;
    else if (isIMUCalibrated && !data.GPSFix && !isLogging)     currentState = READY_NO_GPS;
    else if (isIMUCalibrated && data.GPSFix && !isLogging)      currentState = READY_GPS;
    else if (isIMUCalibrated && !data.GPSFix && isLogging)      currentState = LOGGING_NO_GPS;
    else if (isIMUCalibrated && data.GPSFix && isLogging)       currentState = LOGGING_GPS;
    else                                                        currentState = ERROR_STATE;
}

void updateSystemLED() {
    switch (currentState) {
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
            pixel.setPixelColor(0, AMBER); pixel.show(); // Glow solid amber
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
    Serial.printf("Battery Voltage:             %0.3f V\n\r", data.voltage);
    Serial.printf("Month:                       %d\n\r", data.month);
    Serial.printf("Day:                         %d\n\r", data.day);
    Serial.printf("Year:                        %d\n\r", data.year);
    Serial.printf("Timestamp:                   %s\n\r", data.timestamp);
    Serial.printf("GPS Fix:                     %s\n\r", data.GPSFix ? "true" : "false");
    // Serial.printf("Number of Satellites:        %d\n\r", data.numSats);
    Serial.printf("HDOP:                        %d\n\r", data.HDOP);
    Serial.printf("Latitude:                    %0.6f°\n\r", data.longitude/1E6);
    Serial.printf("Longitude:                   %0.6f°\n\r", data.latitude/1E6);
    Serial.printf("GPS Speed:                   %0.3f kts\n\r", data.GPSSpeed/1E3);
    Serial.printf("GPS Course:                  %0.3f°\n\r", data.GPSCourse);
    Serial.printf("System Calibration:          %d\n\r", data.sysCal);
    Serial.printf("Gyroscope Calibration:       %d\n\r", data.gyroCal);
    Serial.printf("Accelerometer Calibration:   %d\n\r", data.accelCal);
    Serial.printf("Magnetometer Calibration:    %d\n\r", data.magCal);
    // Serial.printf("Acceleration X:              %0.3f m/s/s\n\r", data.accelX);
    // Serial.printf("             Y:              %0.3f m/s/s\n\r", data.accelY);
    // Serial.printf("             Z:              %0.3f m/s/s\n\r", data.accelZ);
    // Serial.printf("Gyroscope X:                 %0.3f rad/s\n\r", data.gyroX);
    // Serial.printf("          Y:                 %0.3f rad/s\n\r", data.gyroY);
    // Serial.printf("          Z:                 %0.3f rad/s\n\r", data.gyroZ);
    Serial.printf("Roll:                        %0.3f°\n\r", data.roll);
    Serial.printf("Pitch:                       %0.3f°\n\r", data.pitch);
    Serial.printf("Yaw:                         %0.3f°\n\r", data.yaw);
    Serial.printf("Linear Acceleration X:       %0.3f m/s/s\n\r", data.linAccelX);
    Serial.printf("                    Y:       %0.3f m/s/s\n\r", data.linAccelY);
    Serial.printf("                    Z:       %0.3f m/s/s\n\r", data.linAccelZ);
    // Serial.printf("Quaternion W:                %0.3f\n\r", data.quatW);
    // Serial.printf("           X:                %0.3f\n\r", data.quatX);
    // Serial.printf("           Y:                %0.3f\n\r", data.quatY);
    // Serial.printf("           Z:                %0.3f\n\r", data.quatZ);
    // Serial.printf("IMU Temperature:             %0.3f°C\n\r", data.imuTemp);
    Serial.printf("Thetis State:                %s\n\r", _statestr);
    Serial.printf("Packet Size:                 %d\n\r", data.packetSize);
    Serial.print("\n\n\r");
}