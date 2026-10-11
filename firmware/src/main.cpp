
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float roll = 0.0f;

// ===== COMPLEMENTARY FILTER =====
const float alpha = 0.98f;
const unsigned long sampleTime = 10000; // 10ms = 100Hz

unsigned long lastTime = 0;
bool firstAngle = true;

// ===== GYRO OFFSET (rad/s) =====
const float gyroOffsetX = -0.11984f;
const float gyroOffsetY = -0.03613f;
const float gyroOffsetZ = -0.00899f;

// ===== ACCEL CALIB =====
const float accelOffset[3] = {
  0.05343f, 0.23715f, 0.64743f
};

const float accelScale[3] = {
  1.005877f, 0.998314f, 0.972981f
};

// CONSTANT FOR PID
float targetRoll = 0.0f;
float Kp = 1.0f;
float Kd = 0.1f;
void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Khong tim thay MPU6050!");
    while (1) delay(100);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 READY");

  lastTime = micros();
}

void loop() {

  // ===== SAMPLE TIME =====
  unsigned long now = micros();

  if ((unsigned long)(now - lastTime) < sampleTime) {
    return;
  }

  float dt = (now - lastTime) * 1e-6f;
  lastTime = now;

  // ===== READ MPU6050 =====
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // ===== GYRO (deg/s) =====
  float p = (g.gyro.x - gyroOffsetX) * RAD_TO_DEG;
  float q = (g.gyro.y - gyroOffsetY) * RAD_TO_DEG;
  float r = (g.gyro.z - gyroOffsetZ) * RAD_TO_DEG;

  // ===== ACCEL (m/s2) =====
  float ax = (a.acceleration.x - accelOffset[0]) * accelScale[0];
  float ay = (a.acceleration.y - accelOffset[1]) * accelScale[1];
  float az = (a.acceleration.z - accelOffset[2]) * accelScale[2];

  // ===== ROLL FROM ACCEL =====
  float roll_a = atan2(
    ay, sqrt(ax * ax + az * az)
  ) * RAD_TO_DEG;

  // ===== INITIAL ANGLE =====
  if (firstAngle) {
    roll = roll_a;
    firstAngle = false;
  }

  // ===== ROLL FROM GYRO =====
  float roll_g = roll + p * dt;

  // ===== COMPLEMENTARY FILTER =====
  roll = alpha * roll_g
       + (1.0f - alpha) * roll_a;

  // ===== TELEPLOT =====
  Serial.printf(">Roll_Accel:%.2f\n", roll_a);
  Serial.printf(">Roll_Gyro:%.2f\n", roll_g);
  Serial.printf(">Roll_Filter:%.2f\n", roll);
}
