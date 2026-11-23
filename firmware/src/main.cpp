
#include <Arduino.h>
#include "web_tune.h"
#include "config.h"
#include "imu.h"
#include "pid.h"
#include "motor.h"

// ===== CONTROL SETTINGS =====
float targetRoll = 0.0f;

const float MAX_ANGLE = 30.0f;
const float ARM_ANGLE = 5.0f;

unsigned long lastTime = 0;
unsigned long lastPrint = 0;

bool motorEnabled = false;
bool autoArm = true;

void setup() {
    Serial.begin(115200);

    if (!imuInit()) {
        Serial.println("MPU6050 ERROR!");
        while (1) delay(100);
    }

    motorInit();
    motorStop();
    webTuneInit();
    lastTime = micros();
}

void loop() {
    webTuneHandle();

    // ===== SAMPLE TIME =====
    unsigned long now = micros();
    unsigned long elapsed = now - lastTime;

    if (elapsed < SAMPLE_TIME_US) {
        return;
    }

    float dt = elapsed * 1e-6f;
    lastTime += (elapsed / SAMPLE_TIME_US) * SAMPLE_TIME_US;

    // ===== READ IMU =====
    IMUData imu = imuUpdate(dt);

    // ===== SAFETY CHECK =====
    if (!isfinite(imu.roll) ||
    !isfinite(imu.gyroRateX) ||
    dt > 0.05f ||
    fabsf(imu.roll - targetRoll) > MAX_ANGLE) {

    motorEnabled = false;
    autoArm = false;
}

    // ===== SERIAL COMMAND =====
  while (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'x' || cmd == 'X') {
        motorEnabled = false;
        autoArm = false;
        motorStop();
    }

    if (cmd == 's' || cmd == 'S') {
        autoArm = true;
    }
}
    // Tu dong bat PD sau khi reset ESP32
if (autoArm && !motorEnabled) {
    if (isfinite(imu.roll) &&
        isfinite(imu.gyroRateX) &&
        dt <= 0.05f &&
        fabsf(imu.roll - targetRoll) < ARM_ANGLE) {

        motorEnabled = true;
        autoArm = false;

        Serial.println("PD AUTO ENABLED");
    }
}

    // ===== PD CONTROLLER =====
    float output = 0.0f;

    if (motorEnabled) {
        output = calculatePD(
            targetRoll,
            imu.roll,
            imu.gyroRateX
        );

        if (!isfinite(output)) {
            motorEnabled = false;
            output = 0.0f;
        }
    }

    // ===== MOTOR CONTROL =====
 int pwm = 0;

if (motorEnabled) {
    output = constrain(output, -255.0f, 255.0f);

    pwm = compensateDeadzone(output);
    setMotor(pwm, pwm);
} else {
    motorStop();
}
}
