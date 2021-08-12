#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);


void setup() {
    Serial.begin(115200);

    //===SSD1306 INITIALIZATION===
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        while(1); //Block further code execution
    }
    // Clear the buffer. By default there is an Adafruit splashscreen loaded into the buffer at boot
    display.clearDisplay();
    display.display();

    display.setFont();
    display.setTextColor(SSD1306_WHITE);

    //===BNO055 INITIALIZATION===
    if(!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    bno.setExtCrystalUse(true);
}

void loop() {
    // //===GET BNO DATA===
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // /* Display the floating point data */
    // Serial.print("aX: "); Serial.print(accel.x());
    // Serial.print(" aY: "); Serial.print(accel.y());
    // Serial.print(" aZ: "); Serial.print(accel.z());
    // Serial.print("\t\t");

    // /* Display the floating point data */
    // Serial.print("gX: "); Serial.print(gyro.x());
    // Serial.print(" gY: "); Serial.print(gyro.y());
    // Serial.print(" gZ: "); Serial.print(gyro.z());
    // Serial.print("\t\t");
    
    // /* Display the floating point data */
    // Serial.print("eX: "); Serial.print(euler.x());
    // Serial.print(" eY: "); Serial.print(euler.y());
    // Serial.print(" eZ: "); Serial.print(euler.z());
    // Serial.print("\t\t");

    // /* Display the floating point data */
    // Serial.print("lX: "); Serial.print(linaccel.x());
    // Serial.print(" lY: "); Serial.print(linaccel.y());
    // Serial.print(" lZ: "); Serial.print(linaccel.z());
    // Serial.print("\t\t");

    // /* Display calibration status for each sensor. */
    uint8_t system, gyro_cal, accel_cal, mag_cal = 0;
    bno.getCalibration(&system, &gyro_cal, &accel_cal, &mag_cal);
    // Serial.print("CALIBRATION: Sys=");
    // Serial.print(system, DEC);
    // Serial.print(" Gyro=");
    // Serial.print(gyro_cal, DEC);
    // Serial.print(" Accel=");
    // Serial.print(accel_cal, DEC);
    // Serial.print(" Mag=");
    // Serial.println(mag_cal, DEC);

    // Display data to OLED
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("CALIBRATE");

    display.setTextSize(1);
    //Calibration data
    display.print(" Gyro=");
    display.print(gyro_cal, DEC);
    display.print(" Accel=");
    display.print(accel_cal, DEC);
    display.print(" Mag=");
    display.println(mag_cal, DEC);

    // //Acceleration data
    // display.print("aX: "); display.print(accel.x());
    // display.print(" aY: "); display.print(accel.y());
    // display.print(" aZ: "); display.print(accel.z());
    // display.println();

    // //Gyroscope data
    // display.print("gX: "); display.print(gyro.x());
    // display.print(" gY: "); display.print(gyro.y());
    // display.print(" gZ: "); display.print(gyro.z());
    // display.println();

    //Orientation data
    display.println();
    display.print("Yaw:   "); display.print(euler.x()); display.println(" deg");
    display.print("Roll:  "); display.print(euler.y()); display.println(" deg");
    display.print("Pitch: "); display.print(euler.z()); display.println(" deg");
    display.println();

    //Linear acceleration data
    // display.print("lX: "); display.print(linaccel.x());
    // display.print(" lY: "); display.print(linaccel.y());
    // display.print(" lZ: "); display.print(linaccel.z());
    // display.println();

    display.display();

    delay(BNO055_SAMPLERATE_DELAY_MS);
    display.clearDisplay();
}