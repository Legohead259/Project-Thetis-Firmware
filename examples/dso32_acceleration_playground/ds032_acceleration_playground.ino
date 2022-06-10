#include <ThetisLib.h>
#include <utility/imumaths.h>

Telemetry data;
bool isDSO32Available = false;

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for serial connection

    Serial.println("-------------------------------------------------");
    Serial.println("        Project Thetis Accelerometer Tests       ");
    Serial.println("-------------------------------------------------");
    Serial.println();

    // IMU Initialization
    Serial.print("Initializing IMU...");
    isDSO32Available = initLSM6DSO32();
    if (!isDSO32Available) {
        Serial.println("Failed!");
        while (true); // Halt further code execution
    }
    Serial.println("done!");

    // Filter Initialization
    Serial.println("Initializing filter...");
    filter.begin(52); // Initialize the filter to expect updates at 52 Hz
    Serial.println("done!");
}

void loop() {
    pollDSO32();
    delay(1000/52.0); // Run at 52 Hz - DSO32 defualt sampling speed
}


// ==============================
// === ACCELERATION FUNCTIONS ===
// ==============================


void pollDSO32() {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    DSO32_IMU.getEvent(&accel, &gyro, &temp);

    float _aX = accel.acceleration.x;
    float _aY = accel.acceleration.y;
    float _aZ = accel.acceleration.z;
    float _gX = gyro.gyro.x;
    float _gY = gyro.gyro.y;
    float _gZ = gyro.gyro.z;

    // Update data packet
    data.accelX = _aX;
    data.accelY = _aY;
    data.accelZ = _aZ;
    data.gyroX = _gX;
    data.gyroY = _gY;
    data.gyroZ = _gZ;
    data.imuTemp = temp.temperature;

    // Debug print statements
    Serial.printf("Accel X: %0.3f \tY: %0.3f \tZ: %0.3f m/s/s\n\r", data.accelX, data.accelY, data.accelZ);
    Serial.printf(" Gyro X: %0.3f \tY: %0.3f \tZ: %0.3f rad/s\n\r", data.gyroX, data.gyroY, data.gyroZ);
    // Serial.printf("   Roll: %0.3f \tP: %0.3f \tY: %0.3f deg\n\r", data.roll, data.pitch, data.yaw);
    Serial.printf("Temperature: %0.3f °C\n\n\r", data.imuTemp);
}

void getLinearAcceleration(float *aX, float *aY, float *aZ) {

}