/*
    Modified from:
    Rui Santos
    Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-client-server-wi-fi/
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files.
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    -------------------------------------------------------------------------------------------------

    This sketch was modified for Project Thetis by Braidan Duffy on 12/14/2020 to change the BME sensor originally used to the on-board BNO055
*/

// Import required libraries
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Set your access point network credentials
const char* ssid = "ThetisNet";
const char* password = "123456789";

Adafruit_BNO055 bno; // I2C

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
 
String readTemp() {
    return String(bno.getTemp());
}

String readCalibration() {
    uint8_t system, gyro_cal, accel_cal, mag_cal = 0;
    bno.getCalibration(&system, &gyro_cal, &accel_cal, &mag_cal);
    char calBuf[64];
    sprintf(calBuf, "%d,%d,%d,%d,%d", system, gyro_cal, accel_cal, mag_cal, bno.isFullyCalibrated()); // TODO: Add timestamp
    return calBuf;
}

String readGyroscope() {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    char gyroBuf[64];
    sprintf(gyroBuf, "%f,%f,%f", gyro.x(), gyro.y(), gyro.z()); // TODO: Add timestamp
    return gyroBuf;
}

String readAccelerometer() {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    char accelBuf[64];
    sprintf(accelBuf, "%f,%f,%f", accel.x(), accel.y(), accel.z()); // TODO: Add timestamp
    return accelBuf;
}

String readMagnetometer() {
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    char magBuf[64];
    sprintf(magBuf, "%f,%f,%f", mag.x(), mag.y(), mag.z()); // TODO: Add timestamp
    return magBuf;
}

String readEulerian() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    char eulerBuf[64];
    sprintf(eulerBuf, "%f,%f,%f", euler.x(), euler.y(), euler.z()); // TODO: Add timestamp
    return eulerBuf;
}

String readIMU() {
    return readCalibration() + "," + readGyroscope() + "," + readAccelerometer() + "," + readMagnetometer() + "," + readEulerian(); // TODO: Add timestamp
}

void setup() {
    // Serial port for debugging purposes
    Serial.begin(115200);
    Serial.println();
    
    // Setting the ESP as an access point
    Serial.print("Setting AP (Access Point)…");
    // Remove the password parameter, if you want the AP (Access Point) to be open
    WiFi.softAP(ssid);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", readTemp().c_str());
    });
    server.on("/calibration", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", readCalibration().c_str());
    });
    server.on("/gyroscope", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", readGyroscope().c_str());
    });
    server.on("/accelerometer", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", readAccelerometer().c_str());
    });
    server.on("/magnetometer", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", readMagnetometer().c_str());
    });
    server.on("/imu", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", readIMU().c_str());
    });

    if (!bno.begin()) {
        Serial.println("Could not find a valid BNO055 sensor, check wiring!");
        while (1); // Block further code execution
    }
    
    // Start server
    server.begin();
}
 
void loop() {
  
}
