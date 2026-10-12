#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "config.h"
#include "imu.h"

static Adafruit_MPU6050 mpu;

// ===== FILTER STATE =====
static float roll = 0.0f;
static bool firstAngle = true;

// ===== GYRO CALIB (rad/s) =====
static const float gyroOffsetX = -0.11984f;

// ===== ACCEL CALIB =====
static const float accelOffset[3] = {
    0.05343f, 0.23715f, 0.64743f
};

static const float accelScale[3] = {
    1.005877f, 0.998314f, 0.972981f
};

// ===== INITIALIZE MPU6050 =====
bool imuInit() {
    Wire.begin(MPU_SDA, MPU_SCL);
    Wire.setClock(400000);

    if (!mpu.begin(MPU_ADDR, &Wire)) {
        return false;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    firstAngle = true;

    return true;
}

// ===== UPDATE COMPLEMENTARY FILTER =====
IMUData imuUpdate(float dt) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Gyro X (deg/s)
    float p = (g.gyro.x - gyroOffsetX) * RAD_TO_DEG;

    // Calibrated accelerometer
    float ax = (a.acceleration.x - accelOffset[0])
               * accelScale[0];

    float ay = (a.acceleration.y - accelOffset[1])
               * accelScale[1];

    float az = (a.acceleration.z - accelOffset[2])
               * accelScale[2];

    // Roll from accelerometer
    float roll_a = atan2(
        ay, sqrt(ax * ax + az * az)
    ) * RAD_TO_DEG;

    // Initial angle
    if (firstAngle) {
        roll = roll_a;
        firstAngle = false;
    }

    // Gyro prediction
    float roll_g = roll + p * dt;

    // Complementary filter
    roll = ALPHA * roll_g
         + (1.0f - ALPHA) * roll_a;

    IMUData data;
    data.rollAccel = roll_a;
    data.rollGyro = roll_g;
    data.roll = roll;
    data.gyroRateX = p;

    return data;
}
