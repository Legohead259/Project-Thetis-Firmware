#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO_SAMPLERATE_DELAY_MS (100)
#define SAMPLE_RATE 32 // Hz
#define BNO_RST_PIN 5
#define BNO_SDA_PIN 26
#define BNO_SCL_PIN 33
#define BNO_RST_PIN 5

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) {
	Serial.begin(115200);
	while(!Serial);
	Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

	// /* Initialise the sensor */
	// Wire.begin(26, 33);
	// if(!bno.begin())
	// {
	// 	/* There was a problem detecting the BNO055 ... check your connections */
	// 	Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
	// 	while(1);
	// }

	// delay(1000);

	// /* Display the current temperature */
	// int8_t temp = bno.getTemp();
	// Serial.print("Current Temperature: ");
	// Serial.print(temp);
	// Serial.println(" C");
	// Serial.println("");

	// bno.setExtCrystalUse(true);

	// Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
	initIMU();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // // Possible vector values can be:
  // // - VECTOR_ACCELEROMETER - m/s^2
  // // - VECTOR_MAGNETOMETER  - uT
  // // - VECTOR_GYROSCOPE     - rad/s
  // // - VECTOR_EULER         - degrees
  // // - VECTOR_LINEARACCEL   - m/s^2
  // // - VECTOR_GRAVITY       - m/s^2
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // /* Display the floating point data */
  // Serial.print("X: ");
  // Serial.print(euler.x());
  // Serial.print(" Y: ");
  // Serial.print(euler.y());
  // Serial.print(" Z: ");
  // Serial.print(euler.z());
  // Serial.print("\t\t");

  
  // // Quaternion data
  // imu::Quaternion quat = bno.getQuat();
  // Serial.print("qW: ");
  // Serial.print(quat.w(), 4);
  // Serial.print(" qX: ");
  // Serial.print(quat.x(), 4);
  // Serial.print(" qY: ");
  // Serial.print(quat.y(), 4);
  // Serial.print(" qZ: ");
  // Serial.print(quat.z(), 4);
  // Serial.print("\t\t");
  

  // /* Display calibration status for each sensor. */
  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.print("CALIBRATION: Sys=");
  // Serial.print(system, DEC);
  // Serial.print(" Gyro=");
  // Serial.print(gyro, DEC);
  // Serial.print(" Accel=");
  // Serial.print(accel, DEC);
  // Serial.print(" Mag=");
  // Serial.println(mag, DEC);

  // delay(BNO_SAMPLERATE_DELAY_MS);
  testIMU();
}

void initIMU() {
    Serial.print("Initializing IMU...");
    Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN); // Initialize I2C bus with correct wires
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055");
        while (true); // blinkCode(IMU_ERROR_CODE, RED); // Block further code execution and blink error code
    }
    bno.setExtCrystalUse(true);

    // pinMode(BNO_RST_PIN, OUTPUT);
    // digitalWrite(BNO_RST_PIN, LOW);

    Serial.println("done!"); // DEBUG
}

void testIMU() {
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

	delay(BNO_SAMPLERATE_DELAY_MS);
}
