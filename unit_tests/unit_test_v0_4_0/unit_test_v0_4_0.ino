#include <ThetisLib.h>

// General Instantiation
#define TEST_TIME 10000 // 10 Seconds

// BNO055 Instantiation
Adafruit_BNO055 BNO055_IMU = Adafruit_BNO055(0x28); // Create BNO object with I2C addr 0x28
bool isBNO055Available;

// LSM6DSO32 Instantiation
bool isDSO32Available;

// Filesystem Instantiation

// GPS Instantiation


// NeoPixel Instantiation

void setup() {
    Serial.begin(115200);
    while(!Serial); // wait for Serial port to open

    Serial.println("---------------------------------------");
    Serial.println("        Project Thetis Unit Test       ");
    Serial.println("                 v0.4.0                ");
    Serial.println("---------------------------------------");

    // NeoPixel initialization and testing
    initNeoPixel();
    testNeoPixel();

    // Log Enable initialization and testing
    attachInterrupt(LOG_EN, logButtonISR, CHANGE);
    testLogEnable();

    // GPS Initialization and testing
    if (!initGPS()) while(true) blinkCode(GPS_ERROR_CODE); // Block further code execution if GPS init fails
    testGPS();

    // IMU Initialization
    isBNO055Available = initBNO055(BNO055_IMU, Serial);
    isDSO32Available = initLSM6DSO32();
    if (isDSO32Available) testDSO32();
    if (!isBNO055Available && !isDSO32Available) { // Check if both IMUs are unavailable
        while(true); // Block further code execution
    }

    // Filesystem Initialization
    if (!initSDCard()) {
        while(true); // Block further code execution
    }
    testFS();

    Serial.println("Unit test complete");
}

void loop() {
}


// =============================
// === DEVICE TEST FUNCTIONS ===
// =============================


void testNeoPixel() {
    Serial.println("Testing NeoPixel...");
    Serial.print("Blinking error code...");
    long startTime = millis();
    while(millis() < startTime + TEST_TIME) {
        blinkCode(IMU_ERROR_CODE, RED); 
    }
    Serial.println("done");

    Serial.print("Rainbow...");
    startTime = millis();
    while(millis() < startTime + TEST_TIME) {
        rainbow(); 
        delay(10);
    }
    Serial.println("done");

    Serial.print("Pulsing...");
    startTime = millis();
    while(millis() < startTime + TEST_TIME) {
        pulseLED(BLUE);
        delay(10);
    }
    pixel.setPixelColor(0, OFF);
    pixel.show(); // Turn off NeoPixel
    Serial.println("done");

    Serial.println("done!");
    Serial.println("---------------------------------------");
    Serial.println();
}

void testLogEnable() {
    Serial.println("Testing LOG_EN button...");
    long startTime = millis();
    while(millis() < startTime + TEST_TIME) {
        if (logButtonPressed && !digitalRead(LOG_EN) && millis() >= logButtonStartTime+LOG_PRESS_TIME) { // Check if log button was pressed and has been held for LOG_PRESS_TIME
            isLogging = !isLogging; // Switch logging state
            Serial.printf("Logging is %s!\r\n", isLogging ? "enabled" : "disabled");
            logButtonPressed = false; // Reset the log button pressed flag
        }
        digitalWrite(LED_BUILTIN, isLogging);
    } 
    Serial.println("done!");
    Serial.println("---------------------------------------");
    Serial.println();
}

void testGPS() {
    Serial.println("Testing GPS...");
    long startTime = millis();
    while (millis() < startTime + TEST_TIME) { // For TEST_TIME, read GPS data from bus
        // TODO: REmove dependence on PPS trigger
        static bool _newGPSData = false;
        while (GPS.available()) { // Print out raw GPS data when PPS is not triggered (no 3D fix)
            char c = GPS.read();
            Serial.print(c); // DEBUG
            nmea.process(c);
            _newGPSData = true; // Set the new GPS data flag
	    }

        if (_newGPSData && nmea.isValid()) {
            _newGPSData = false; // Reset the new GPS data flag

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
    }
    Serial.println("done!");
    Serial.println("---------------------------------------");
    Serial.println();
}

void testBNO055() {
    Serial.println("Testing BNO055...");
    long startTime = millis();
    while (millis() < startTime + TEST_TIME) { // For TEST_TIME, read off IMU data at SAMPLE_RATE
        // Possible vector values can be:
        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
        imu::Vector<3> accelerations = BNO055_IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

        // Display the floating point data
        Serial.print("X: ");
        Serial.print(accelerations.x());
        Serial.print(" Y: ");
        Serial.print(accelerations.y());
        Serial.print(" Z: ");
        Serial.print(accelerations.z());
        Serial.print("\t\t");

        // Quaternion data
        imu::Quaternion quat = BNO055_IMU.getQuat();
        Serial.print("qW: ");
        Serial.print(quat.w(), 4);
        Serial.print(" qX: ");
        Serial.print(quat.x(), 4);
        Serial.print(" qY: ");
        Serial.print(quat.y(), 4);
        Serial.print(" qZ: ");
        Serial.print(quat.z(), 4);
        Serial.print("\t\t");

        // Display calibration status for each sensor.
        uint8_t system, gyro, accel, mag = 0;
        BNO055_IMU.getCalibration(&system, &gyro, &accel, &mag);
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

void testDSO32() {
    long _startTime = millis();
    while(millis() <= _startTime+TEST_TIME) {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        DSO32_IMU.getEvent(&accel, &gyro, &temp);

        Serial.printf("Accel X: %0.3f \tY: %0.3f \tZ: %0.3f m/s/s\n\r", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
        Serial.printf(" Gyro X: %0.3f \tY: %0.3f \tZ: %0.3f rad/s\n\r", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
        Serial.printf("Temperature: %0.3f °C\n\n\r", temp.temperature);

        delay(125);
    }
    Serial.println("done!");
    Serial.println("---------------------------------------");
    Serial.println();
}

void testFS() {
    Serial.println("Testing filesystem...");
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
