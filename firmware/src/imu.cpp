#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "config.h"
#include "imu.h"

// Shared MPU6050 driver used for initialization and sensor reads.
static Adafruit_MPU6050 mpu;

// Complementary-filter state; roll persists between samples for gyro integration.
static float roll = 0.0f;
static bool firstAngle = true;

// Calibrated X-axis gyro bias in radians/second.
static const float gyroOffsetX = -0.11984f;

// Calibrated accelerometer offsets and scale factors for the X/Y/Z axes.
static const float accelOffset[3] = {
    0.05343f, 0.23715f, 0.64743f
};

static const float accelScale[3] = {
    1.005877f, 0.998314f, 0.972981f
};

// Start I2C and configure the MPU6050 measurement ranges and internal filter.
bool imuInit() {
    Wire.begin(MPU_SDA, MPU_SCL);
    Wire.setClock(400000);

    if (!mpu.begin(MPU_ADDR, &Wire)) {
        return false;
    }

    // Select balance-robot ranges: +/-2 g and +/-250 degrees/second.
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    firstAngle = true;

    return true;
}

// Read one sensor sample, calculate roll, and fuse gyro with accelerometer data.
IMUData imuUpdate(float dt) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Remove gyro bias and convert X angular rate from radians/second to degrees/second.
    float p = (g.gyro.x - gyroOffsetX) * RAD_TO_DEG;

    // Calibrate each accelerometer axis before calculating the angle.
    float ax = (a.acceleration.x - accelOffset[0])
               * accelScale[0];

    float ay = (a.acceleration.y - accelOffset[1])
               * accelScale[1];

    float az = (a.acceleration.z - accelOffset[2])
               * accelScale[2];

    // Accelerometer provides an absolute roll estimate, but is noisy during vibration.
    float roll_a = atan2(
        ay, sqrt(ax * ax + az * az)
    ) * RAD_TO_DEG;

    // Initialize the filter from the accelerometer to avoid starting from an incorrect zero.
    if (firstAngle) {
        roll = roll_a;
        firstAngle = false;
    }

    // Predict roll with the gyro; this responds quickly but drifts if used alone.
    float roll_g = roll + p * dt;

    // Complementary filter: gyro follows fast changes; accelerometer corrects long-term drift.
    roll = ALPHA * roll_g
         + (1.0f - ALPHA) * roll_a;

    // Package the sample; the 2.5-degree offset calibrates the robot's upright position.
    IMUData data;
    data.rollAccel = roll_a;
    data.rollGyro = roll_g;
    data.roll = roll + 2.5f;
    data.gyroRateX = p;
    data.accelX = ax;
    data.accelY = ay;
    data.accelZ = az;
    data.gyroRateY = g.gyro.y * RAD_TO_DEG;
    data.gyroRateZ = g.gyro.z * RAD_TO_DEG;

    return data;
}
