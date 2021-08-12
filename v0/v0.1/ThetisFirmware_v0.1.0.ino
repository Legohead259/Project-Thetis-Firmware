#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char *ssid     = "Niagra";
const char *password = "travis123";
const long utcOffsetInSeconds = -6*3600;
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
int lastSecond = 0;
float lastMillis = millis();
float curMillis = 0;
float curMSecond = 0;
int currentDay, currentMonth, currentYear = 0;

#define SD_CS 15            // Chip select on ESP8266 is 15
File logFile;
char curFilename[32];
char timestamp[32];

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET    -1    // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
uint8_t sysCal, gyroCal, accelCal, magCal = 0;
bool isCalibrated = false;
imu::Vector<3> accel;
imu::Vector<3> gyro;
imu::Vector<3> euler;
imu::Vector<3> linaccel;

enum state {
    ERROR = -1,
    BOOTING,
    CALIBRATION,
    RECORDING
};

uint8_t curState = BOOTING;

void setup() {
    Serial.begin(115200);

    initSDCard();
    initOLED();
    initBNO055();

    curState = CALIBRATION;
}

void loop() {
    pollIMU();
    while (!checkIsCalibrated()) { //Block code execution until IMU is calibrated
        curState = CALIBRATION;
        displayCalibrationStatus();
    }
    curState = RECORDING;
    displayIMUData();
    logIMUData();
}

void displayCalibrationStatus() {
    // Display data to OLED
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("CALIBRATE");

    display.setTextSize(1);
    //Calibration data
    display.printf(" Gyro=%d", gyroCal);
    display.printf(" Accel=%d", accelCal);
    display.printf(" Mag=%d", magCal);

    display.display();
    delay(BNO055_SAMPLERATE_DELAY_MS);
    display.clearDisplay();
}

void displayIMUData() {
    // Display data to OLED
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("IMU DATA");

    display.setTextSize(1);
    //Calibration data
    display.printf(" Gyro=%d", gyroCal);
    display.printf(" Accel=%d", accelCal);
    display.printf(" Mag=%d", magCal);
    display.println();
    display.print("Yaw:   "); display.print(euler.x()); display.println(" deg");
    display.print("Roll:  "); display.print(euler.y()); display.println(" deg");
    display.print("Pitch: "); display.print(euler.z()); display.println(" deg");
    display.println();

    display.display();
    delay(BNO055_SAMPLERATE_DELAY_MS);
    display.clearDisplay();
}

void logIMUData() {
    updateTime();
    logFile = SD.open(curFilename, FILE_WRITE);
    logFile.printf("%s,%f,%f,%f \n", timestamp, euler.y(), euler.z(), euler.x()); //Example: 15:21:23:255,ROLL,PITCH,YAW
    logFile.close();
}

void initWiFi() {
    WiFi.begin(ssid, password);

    while ( WiFi.status() != WL_CONNECTED ) { //Block code execution until WiFi connection is made
        delay(500); //Check connection every 500 ms
        Serial.print(".");
    }

    timeClient.begin(); //Connect to time client
}

void initSDCard() {
    Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CS)) { // Initialize SD card
        Serial.println("No SD Card detected");
        while(1); //Block further code execution
    }
    Serial.println("done!");
    
    Serial.print("Creating log file...");
    updateTime();
    if (!createFile(curFilename, 11, 14, 2020)) {
        Serial.println("Failed to create log file");
        while(1); //Block further code execution
    }
    Serial.println("done!");
    Serial.printf("Created file: %s \n", curFilename);
}

bool createFile(char* filename, uint8_t m, uint8_t d, uint16_t y) {
    for (int x=0; x<100; x++) {
        sprintf(filename, "/datalog_%02d%02d%02d_%02d.csv", m, d, y, x); // Example name: datalog_10302020_001.csv
        if (!SD.exists(filename)) {
            Serial.print("Filename available: "); Serial.println(filename);
            logFile = SD.open(filename, FILE_WRITE);
            if (!logFile) {
                Serial.print("Couldn't create "); Serial.println(filename);
                return false;
            }
            logFile.println("Timestamp (hh:mm:ss:mss),roll (deg),pitch (deg),yaw (deg)"); //DEBUG
            logFile.close();
            return true;
        }
    }
    return false;
}

void initOLED() {
    Serial.print("Initializing OLED screen...");
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
        Serial.println("SSD1306 not initialized");
        while(1); //Block further code execution
    }
    // Clear the buffer. By default there is an Adafruit splashscreen loaded into the buffer at boot
    display.clearDisplay();
    display.display();

    display.setFont();
    display.setTextColor(SSD1306_WHITE);
    Serial.println("done!");
}

void initBNO055() {
    Serial.print("Initializing BNO055 IMU...");
    if(!bno.begin()) {
        Serial.print("BNO055 not initialized");
        while(1); //Block further code execution
    }
    bno.setExtCrystalUse(true);
    Serial.println("done!");
}

void pollIMU() {
    bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
    accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

void updateTime() {
    timeClient.update();

    //Parse milliseconds for timestamp
    int curSecond = timeClient.getSeconds();
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
    sprintf(timestamp, "%02d:%02d:%02d:%03d", timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds(), (int) curMSecond);
    // Serial.println(timestamp); //DEBUG

    unsigned long epochTime = timeClient.getEpochTime();
    //Get a time structure
    struct tm *ptm = gmtime ((time_t *)&epochTime); 
    currentDay = ptm->tm_mday; //Get the day from the time structure
    currentMonth = ptm->tm_mon+1; //Get the month from the time structure
    currentYear = ptm->tm_year+1900; //Get the year from the time structure. Note: structure is year since 1900, so need to add 1900
}

bool checkIsCalibrated() {
    bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
    return gyroCal == 3 && accelCal == 3 && magCal == 3;
}