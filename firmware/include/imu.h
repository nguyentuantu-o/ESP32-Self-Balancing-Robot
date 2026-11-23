#pragma once

// Sensor values calculated from the MPU and shared with the control loops.
struct IMUData {
    float rollAccel;  // Roll from the accelerometer alone, degrees
    float rollGyro;   // Gyro-predicted roll, degrees
    float roll;       // Complementary-filtered roll with calibration offset, degrees
    float gyroRateX;  // Angular rate used by the balance loop, degrees/second
    // Remaining sensor axes, available for monitoring and display.
    float accelX;
    float accelY;
    float accelZ;
    float gyroRateY;
    float gyroRateZ;
};

// Initialize I2C and the MPU6050; return false if the sensor is not found.
bool imuInit();

// Read the MPU, filter roll, and return the current sample; dt is in seconds.
IMUData imuUpdate(float dt);
