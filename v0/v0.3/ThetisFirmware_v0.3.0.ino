#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <RTClib.h>
#include <Adafruit_DotStar.h>

#define SAMPLE_RATE 32 // Hz

#define SD_CS A5

#define IMU_RST 9

#define NUM_PIXELS           1
#define DOTSTAR_CLK_PIN     40
#define DOTSTAR_DATA_PIN    41

// SD Card instantiation
File dataFile;
char filename[30];
Adafruit_USBD_MSC usb_msc;
Sd2Card card;
SdVolume volume;

// BNO055 instantiation
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// RTC instatiation
RTC_DS3231 rtc;
uint8_t lastSecond = 0;
float lastMillis = millis();
float curMillis = 0;
float curMSecond = 0;

// DotStar instantiation
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

// General instantiation
struct packet {
    char timestamp[32];
    uint8_t sysCal;
    uint8_t accelCal;
    uint8_t gyroCal;
    uint8_t magCal;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float roll;
    float pitch;
    float yaw;
    float linAccelX;
    float linAccelY;
    float linAccelZ;
};
struct packet data; 

void setup() {
    Serial.begin(115200);
    // while (!Serial); // Wait for serial connection - DEBUG
    strip.setPixelColor(0, OFF); strip.show();

    // Initialize USB MSC
    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
    usb_msc.setID("FL-TECH", "Thetis", "r.F1");

    // Set read write callback
    usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

    // Still initialize MSC but tell usb stack that MSC is not ready to read/write
    // If we don't initialize, board will be enumerated as CDC only
    usb_msc.setUnitReady(false);
    usb_msc.begin();

    // Initialize SD Card
    Serial.print("Initializing SD Card...");
    if (!SD.begin(SD_CS)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        strip.setPixelColor(0, RED); strip.show();// Indicate failure on the NeoPixel
        while (1);
    }
    Serial.println("SD card initialized!");

    // Initialize BNO055
    Serial.print("Initializing IMU..."); // DEBUG
    if (!bno.begin()) {
        Serial.println("BNO failed or not present"); // DEBUG
        strip.setPixelColor(0, RED); strip.show(); // Indicate failure on the NeoPixel
        while (1); // Don't do anything else
    }
    bno.setExtCrystalUse(true);
    Serial.println("IMU initialized!"); // DEBUG

    // Initialize RTC
    Serial.print("Initializing RTC..."); // DEBUG
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC"); // DEBUG
        strip.setPixelColor(0, RED); strip.show(); // Indicate failure on the NeoPixel
        while (1); // Don't do anything else
    }
    Serial.println("RTC initialized!"); // DEBUG
    if (rtc.lostPower()) {
        Serial.println("RTC lost power, setting to compile time and date");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    // Create data file
    DateTime now = rtc.now();
    for (int x=0; x<100; x++) {
        sprintf(filename, "%02d%02d_%02d.csv", now.month(), now.day(), x); // ATM the filename can only be 8 chars long
        if (!SD.exists(filename)) {
            break;
        }
        Serial.println(filename); //DEBUG
    }
    dataFile = SD.open(filename, FILE_WRITE);
    if (!dataFile) { // Check data file exists
        Serial.print("Couldn't create ");
        Serial.println(filename);
        Serial.println("Halting...");
        strip.setPixelColor(0, RED); strip.show(); // Indicate failure on the NeoPixel
        while(1);
    }
    else { // Write headers
        dataFile.print("Timestamp (ISO8601)"); dataFile.print(",");
        dataFile.print("sys_cal"); dataFile.print(",");
        dataFile.print("gyro_cal"); dataFile.print(",");
        dataFile.print("accel_cal"); dataFile.print(",");
        dataFile.print("mag_cal"); dataFile.print(",");
        dataFile.print("accel_x"); dataFile.print(",");
        dataFile.print("accel_y"); dataFile.print(",");
        dataFile.print("accel_z"); dataFile.print(",");
        dataFile.print("gyro_x"); dataFile.print(",");
        dataFile.print("gyro_y"); dataFile.print(",");
        dataFile.print("gyro_z"); dataFile.print(",");
        dataFile.print("roll"); dataFile.print(",");
        dataFile.print("pitch"); dataFile.print(",");
        dataFile.print("yaw"); dataFile.print(",");
        dataFile.print("linAccel_x"); dataFile.print(",");
        dataFile.print("linAccel_y"); dataFile.print(",");
        dataFile.print("linAccel_z"); dataFile.print(",");
        dataFile.println(); // End line entry

        dataFile.close();
    }
    Serial.print("Writing to: "); Serial.println(filename); // DEBUG
}

void loop() {
    //==========================
    //=====FORMAT TIMESTAMP=====
    //==========================

    DateTime now = rtc.now();

    //Parse milliseconds for timestamp
    uint8_t curSecond = now.second();
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
    // Format timestamp in Lowell instrument's ISO8601 format
    sprintf(data.timestamp, "%04d-%02d-%02dT%02d:%02d:%02d.%03d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), (int) curMSecond);
    // Serial.println(data.timestamp); // DEBUG

    //==================
    //=====POLL IMU=====
    //==================

    //Get calibration status for each sensor.
    bno.getCalibration(&data.sysCal, &data.gyroCal, &data.accelCal, &data.magCal);

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

    //================================
    //=====LOG DATA PACKET TO FILE====
    //================================

    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        dataFile.print(data.timestamp); dataFile.print(",");
        dataFile.print(data.sysCal); dataFile.print(",");
        dataFile.print(data.gyroCal); dataFile.print(",");
        dataFile.print(data.accelCal); dataFile.print(",");
        dataFile.print(data.magCal); dataFile.print(",");
        dataFile.print(data.accelX); dataFile.print(",");
        dataFile.print(data.accelY); dataFile.print(",");
        dataFile.print(data.accelZ); dataFile.print(",");
        dataFile.print(data.gyroX); dataFile.print(",");
        dataFile.print(data.gyroY); dataFile.print(",");
        dataFile.print(data.gyroZ); dataFile.print(",");
        dataFile.print(data.roll); dataFile.print(",");
        dataFile.print(data.pitch); dataFile.print(",");
        dataFile.print(data.yaw); dataFile.print(",");
        dataFile.print(data.linAccelX); dataFile.print(",");
        dataFile.print(data.linAccelY); dataFile.print(",");
        dataFile.print(data.linAccelZ); dataFile.print(",");
        dataFile.println(); // End line entry

        dataFile.close();
    }
    else {
        Serial.println("Unable to open datafile");
        strip.setPixelColor(0, RED); strip.show(); // Indicate failure on the NeoPixel
        while(1); // Do nothing else
    }

    //=================================
    //=====NEOPIXEL STATUS DISPLAY=====
    //=================================

    if (bno.isFullyCalibrated()) {
        strip.setPixelColor(0, GREEN); strip.show(); // Display Green when system calibrated
    }
    else if (data.gyroCal < 3) {
        strip.setPixelColor(0, BLUE); strip.show(); // Display Blue when waiting on gyro calibration
    }
    else if (data.accelCal < 3) {
        strip.setPixelColor(0, PURPLE); strip.show(); // Display Purple when waiting on accelerometer calibration
    }
    else if (data.magCal < 3) {
        strip.setPixelColor(0, LIME); strip.show(); // Display Lime when waiting on magnetometer calibration
    }
    else if (data.sysCal < 3) {
        strip.setPixelColor(0, WHITE); strip.show(); // Display white when waiting on system calibration
    }

    delay(1/SAMPLE_RATE);
}
