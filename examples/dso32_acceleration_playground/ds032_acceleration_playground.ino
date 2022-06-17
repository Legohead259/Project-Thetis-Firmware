#include <ThetisLib.h>
#include <utility/imumaths.h>
#include <filters/ButterworthBP2.h>

#include <Kalman.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;
uint32_t timer;

telemetry_t data;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
sensors_vec_t accelAngle;
sensors_vec_t ufAngle;
sensors_vec_t cfAngle;
sensors_vec_t kfAngle;

bool isDSO32Available = false;
// const double f_c = 1; // Hz - cutoff frequency
// const double f_s = 52; // Hz - sample frequency
// const double f_n = 2 * f_c / f_s; // Normalized cutoff frequency
// auto filter = butter<6>(f_n); // Creates a 6th-order Butterworth filter with a normalized cutoff frequency

void calcLinAccel(sensors_vec_t &linAccel, sensors_vec_t &accel, double fc=1, double fs=52);
void computeCFAngles(sensors_vec_t &angle, sensors_vec_t & accel, sensors_vec_t &gyro, double dt,
                     double wGyro=0.93, double wAccel=0.07);

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for serial connection

    Serial.println("-------------------------------------------------");
    Serial.println("        Project Thetis Accelerometer Tests       ");
    Serial.println("-------------------------------------------------");
    Serial.println();

    // IMU Initialization
    isDSO32Available = initLSM6DSO32();
    if (!isDSO32Available) {
        while (true); // Halt further code execution
    }

    pollDSO32();
    computeRawAngles(ufAngle, accel.acceleration, gyro.gyro, 0);

    // Set starting angles
    cfAngle.roll = ufAngle.roll;
    cfAngle.pitch = ufAngle.pitch;
    cfAngle.heading = ufAngle.heading;
    kalmanX.setAngle(ufAngle.roll); 
    kalmanY.setAngle(ufAngle.pitch);
    kalmanZ.setAngle(ufAngle.heading);

    timer = micros();
}

void loop() {
    double dt = (double)(micros() - timer) / 1E6; // Calculate delta time
    pollDSO32();
    
    // sensors_vec_t linAccel;
    // calcLinAccel(linAccel, accel.acceleration);
    computeAngles(accel.acceleration, gyro.gyro, dt);
    // Serial.println();
    timer = micros();
    delay(1000/52.0); // Run at 52 Hz - DSO32 defualt sampling speed
    // delay(500);
}


// ==============================
// === ACCELERATION FUNCTIONS ===
// ==============================


void pollDSO32() {
    DSO32_IMU.getEvent(&accel, &gyro, &temp);

    // Convert gyro to deg/s from rad/s
    gyro.gyro.x *= RAD_TO_DEG;
    gyro.gyro.y *= RAD_TO_DEG;
    gyro.gyro.z *= RAD_TO_DEG;

    // Update data packet
    data.accelX = accel.acceleration.x;
    data.accelY = accel.acceleration.y;
    data.accelZ = accel.acceleration.z;
    data.gyroX = gyro.gyro.x;
    data.gyroY = gyro.gyro.y;
    data.gyroZ = gyro.gyro.z;
    data.imuTemp = temp.temperature;

    // Debug print statements
    // Serial.printf("Accel X: %0.3f \tY: %0.3f \tZ: %0.3f m/s/s\n\r", data.accelX, data.accelY, data.accelZ);
    // Serial.printf(" Gyro X: %0.3f \tY: %0.3f \tZ: %0.3f rad/s\n\r", data.gyroX, data.gyroY, data.gyroZ);
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

    // Add calculations to data packet
    // data.linAccelX = linAccel.x;
    // data.linAccelY = linAccel.y;
    // data.linAccelZ = linAccel.z;

    // DEBUG statements
    Serial.printf("X: %0.3f \tY: %0.3f \tZ: %0.3f m/s/s\n\r", linAccel.x, linAccel.y, linAccel.z);
}


// =============================
// === ORIENTATION FUNCTIONS ===
// =============================


void computeAngles(sensors_vec_t &accel, sensors_vec_t &gyro, double dt) {
    computeAccelAngles(accelAngle, accel);
    computeRawAngles(ufAngle, accel, gyro, dt);    
    computeCFAngles(cfAngle, accel, gyro, dt);
    computeKFAngles(kfAngle, accel, gyro, dt);

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((accelAngle.roll < -90 && kfAngle.roll > 90) || (accelAngle.roll > 90 && kfAngle.roll < -90)) {
        kalmanX.setAngle(accelAngle.roll);
        ufAngle.roll = accelAngle.roll;
        cfAngle.roll = accelAngle.roll;
        kfAngle.roll = accelAngle.roll;
    } else
        kfAngle.roll = kalmanX.getAngle(accelAngle.roll, gyro.x, dt); // Calculate the angle using a Kalman filter

    if (abs(kfAngle.roll) > 90)
        gyro.y = -gyro.y; // Invert rate, so it fits the restriced accelerometer reading
    kfAngle.pitch = kalmanY.getAngle(accelAngle.pitch, gyro.y, dt);

    // Reset the angle when it has drifted too much
    if (ufAngle.roll < -180 || ufAngle.roll > 180)
        ufAngle.roll = kfAngle.roll;

    if (ufAngle.pitch < -180 || ufAngle.pitch > 180)
        ufAngle.pitch = kfAngle.pitch;

    if (ufAngle.heading < -180 || ufAngle.heading > 180)
        ufAngle.heading = kfAngle.heading;

    // Add calculations to data packet
    uint8_t rollQualities[3] = {25, 75, 75};
    float rollValues[3] = {accelAngle.roll, cfAngle.roll, kfAngle.roll};
    data.roll  = weightedAvg(rollQualities, rollValues, 3);

    uint8_t pitchQualities[3] = {25, 75, 75};
    float pitchValues[3] = {accelAngle.pitch, cfAngle.pitch, kfAngle.pitch};
    data.pitch = weightedAvg(pitchQualities, pitchValues, 3);
    
    data.yaw   = cfAngle.heading;

    // DEBUG statements
    // Serial.printf("Accelerometer Roll: %0.3f \t\t Pitch: %0.3f \t\t Yaw: %0.3f deg\n\r", accelAngle.roll, accelAngle.pitch, accelAngle.heading);
    // Serial.printf("Unfiltered    Roll: %0.3f \t\t Pitch: %0.3f \t\t Yaw: %0.3f deg\n\r", ufAngle.roll, ufAngle.pitch, ufAngle.heading);
    // Serial.printf("Complimentary Roll: %0.3f \t\t Pitch: %0.3f \t\t Yaw: %0.3f deg\n\r", cfAngle.roll, cfAngle.pitch, cfAngle.heading);
    // Serial.printf("Kalman        Roll: %0.3f \t\t Pitch: %0.3f \t\t Yaw: %0.3f deg\n\r", kfAngle.roll, kfAngle.pitch, kfAngle.heading);

    Serial.printf("Roll: %0.3f \t\t Pitch: %0.3f \t\t Yaw: %0.3f deg\n\r", data.roll, data.pitch, data.yaw);
    
    Serial.println();
}

void computeAccelAngles(sensors_vec_t &angle, sensors_vec_t &accel) {
    // Compute orientations using equations from: 
    angle.roll  = atan2(accel.y, accel.z) * RAD_TO_DEG;
    angle.pitch = atan(-accel.x / sqrt(accel.y*accel.y + accel.z*accel.z)) * RAD_TO_DEG;
    angle.heading = atan(sqrt((accel.y*accel.y) +(accel.x*accel.y)) / accel.z) * RAD_TO_DEG;
}

void computeRawAngles(sensors_vec_t &angle, sensors_vec_t &accel, sensors_vec_t &gyro, double dt) {
    // Compute orientations from accelerometer
    sensors_vec_t _accelAngle;
    computeAccelAngles(_accelAngle, accel);

    // Integrate gyro values and add to orientation
    angle.roll    += _accelAngle.roll    + gyro.x * dt;
    angle.pitch   += _accelAngle.pitch   + gyro.y * dt;
    angle.heading += _accelAngle.heading + gyro.z * dt;

    // Add calculations into data packet
    // data.roll  = angle.roll;
    // data.pitch = angle.pitch;
    // data.yaw   = angle.heading;

    // DEBUG statements
    // Serial.printf("Roll: %0.3f \t Pitch: %0.3f \t Yaw: %0.3f deg\n\r", angle.roll, angle.pitch, angle.heading);
}

void computeCFAngles(sensors_vec_t &angle, sensors_vec_t & accel, sensors_vec_t &gyro, double dt,
                     double wGyro, double wAccel) {
    // Check for valid weights, return if not valid
    if (wGyro+wAccel != 1) return;

    // Compute raw orientation angles
    sensors_vec_t _accelAngle;
    computeAccelAngles(_accelAngle, accel);

    // Compute complimentary-filtered angles
    angle.roll    = wGyro * (_accelAngle.roll    + gyro.x * dt) + wAccel * _accelAngle.roll;
    angle.pitch   = wGyro * (_accelAngle.pitch   + gyro.y * dt) + wAccel * _accelAngle.pitch;
    angle.heading = wGyro * (_accelAngle.heading + gyro.y * dt) + wAccel * _accelAngle.heading;

    // Add calculations into data packet
    // data.roll  = angle.roll;
    // data.pitch = angle.pitch;
    // data.yaw   = angle.heading;

    // DEBUG statements
    // Serial.printf("Roll: %0.3f \t Pitch: %0.3f \t Yaw: %0.3f deg\n\r", angle.roll, angle.pitch, angle.heading);
}

void computeKFAngles(sensors_vec_t &angle, sensors_vec_t & accel, sensors_vec_t &gyro, double dt) {
    // Compute orientations from accelerometer
    sensors_vec_t _accelAngle;
    computeAccelAngles(_accelAngle, accel);

    // Compute Kalman-filtered angles
    angle.roll    = kalmanX.getAngle(_accelAngle.roll, gyro.x, dt);
    angle.pitch   = kalmanY.getAngle(_accelAngle.pitch, gyro.y, dt);
    angle.heading = kalmanZ.getAngle(_accelAngle.heading, gyro.z, dt);

    // Add calculations into data packet
    // data.roll  = angle.roll;
    // data.pitch = angle.pitch;
    // data.yaw   = angle.heading;

    // DEBUG statements
    // Serial.printf("Roll: %0.3f \t Pitch: %0.3f \t Yaw: %0.3f deg\n\r", angle.roll, angle.pitch, angle.heading);
}


// =========================
// === UTILITY FUNCTIONS ===
// =========================


float weightedRMS(uint8_t qualities[], float values[], uint8_t n) {
    // Convert qualities into weight
    float _sum = 0;
    for (int i=0; i<n; i++) { // Sum up the qualities
        _sum += qualities[i];
    }
    float _weights[n];
    for (int i=0; i<n; i++) { // Normalize qualities into weights
        _weights[i] = qualities[i]/_sum;
    }

    // wRMS = sqrt(sum(w[] * x[]))
    _sum = 0;
    for (int i=0; i<n; i++) { // Find sum inside sqrt
        _sum += _weights[i] * (values[i]*values[i]);
    }
    return sqrt(_sum); // Calculate weighted RMS
}

float weightedAvg(uint8_t qualities[], float values[], uint8_t n) {
    // Convert qualities into weight
    float _sum = 0;
    for (int i=0; i<n; i++) { // Sum up the qualities
        _sum += qualities[i];
    }
    float _weights[n];
    for (int i=0; i<n; i++) { // Normalize qualities into weights
        _weights[i] = qualities[i]/_sum;
    }

    float _avg = 0;
    for (int i=0; i<n; i++) { // Calculate weight average
        _avg += _weights[i] * values[i];
    }
    return _avg;
}