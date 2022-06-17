#include <ThetisLib.h>
#include <utility/imumaths.h>
#include <filters/ButterworthBP2.h>

telemetry_t data;
bool isDSO32Available = false;
// const double f_c = 1; // Hz - cutoff frequency
// const double f_s = 52; // Hz - sample frequency
// const double f_n = 2 * f_c / f_s; // Normalized cutoff frequency
// auto filter = butter<6>(f_n); // Creates a 6th-order Butterworth filter with a normalized cutoff frequency

void calcLinAccel(sensors_vec_t &linAccel, sensors_vec_t &accel, double fc=1, double fs=52);

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
    // Serial.println("Initializing filter...");
    // filter.begin(52); // Initialize the filter to expect updates at 52 Hz
    // Serial.println("done!");
}

void loop() {
    pollDSO32();
    // delay(1000/52.0); // Run at 52 Hz - DSO32 defualt sampling speed
    delay(500);
}


// ==============================
// === ACCELERATION FUNCTIONS ===
// ==============================


void pollDSO32() {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    DSO32_IMU.getEvent(&accel, &gyro, &temp);
    sensors_vec_t linAccel;
    calcLinAccel(linAccel, accel.acceleration);

    // Update data packet
    data.accelX = accel.acceleration.x;
    data.accelY = accel.acceleration.x;
    data.accelZ = accel.acceleration.x;
    data.linAccelX = linAccel.x;
    data.linAccelY = linAccel.y;
    data.linAccelZ = linAccel.z;
    data.gyroX = gyro.gyro.x;
    data.gyroY = gyro.gyro.x;
    data.gyroZ = gyro.gyro.x;
    data.imuTemp = temp.temperature;

    // Debug print statements
    // Serial.printf("Accel X: %0.3f \tY: %0.3f \tZ: %0.3f m/s/s\n\r", data.accelX, data.accelY, data.accelZ);
    // Serial.printf(" Gyro X: %0.3f \tY: %0.3f \tZ: %0.3f rad/s\n\r", data.gyroX, data.gyroY, data.gyroZ);
    // Serial.printf("LinAc X: %0.3f \tY: %0.3f \tZ: %0.3f m/s/s\n\r", data.linAccelX, data.linAccelY, data.linAccelZ);
    // Serial.printf("   Roll: %0.3f \tP: %0.3f \tY: %0.3f deg\n\r", data.roll, data.pitch, data.yaw);
    // Serial.printf("Temperature: %0.3f °C\n\n\r", data.imuTemp);

    // Serial plotter print statements
    // Serial.printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n\r",data.accelX, data.accelY, data.accelZ, 
    //                                                         data.linAccelX, data.linAccelY, data.linAccelZ);
}

void calcLinAccel(sensors_vec_t &linAccel, sensors_vec_t &accel, double fc, double fs) {
    const double fn = 2 * fc / fs; // Normalized cut-off frequency
    ButterworthBP2 filter;
    
    // Calculate the gravity vector from the low-pass Butterworth filter
    sensors_vec_t gravity;
    gravity.x = filter.step(accel.x);
    gravity.y = filter.step(accel.y);
    gravity.z = filter.step(accel.z);

    // Calculate the linear acceleration by removing the gravity signal
    linAccel.x = accel.x - gravity.x;
    linAccel.y = accel.y - gravity.y;
    linAccel.z = accel.z - gravity.z;

    Serial.printf("Accel   X: %0.3f \tY: %0.3f \tZ: %0.3f m/s/s\n\r", accel.x, accel.y, accel.z);
    Serial.printf("Gravity X: %0.3f \tY: %0.3f \tZ: %0.3f m/s/s\n\r", gravity.x, gravity.y, gravity.z);
    Serial.printf("LinAcc  X: %0.3f \tY: %0.3f \tZ: %0.3f m/s/s\n\r", linAccel.x, linAccel.y, linAccel.z);
}