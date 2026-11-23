#pragma once
struct IMUData {
    float rollAccel;  // Roll tu accelerometer
    float rollGyro;   // Roll du doan tu gyro
    float roll;       // Roll sau complementary filter
    float gyroRateX;  // Toc do goc X (deg/s)
};

bool imuInit();
IMUData imuUpdate(float dt);
