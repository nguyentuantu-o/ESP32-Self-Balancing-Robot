
#include <Arduino.h>

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

void setup() {
    Serial.begin(115200);

    if (!imuInit()) {
        Serial.println("MPU6050 ERROR!");
        while (1) delay(100);
    }

    motorInit();
    motorStop();

    Serial.println("SYSTEM READY");
    Serial.println("Gui S de bat PD khi xe gan thang.");
    Serial.println("Gui X de dung motor.");

    lastTime = micros();
}

void loop() {

    // ===== SAMPLE TIME =====
    unsigned long now = micros();

    if ((unsigned long)(now - lastTime) < SAMPLE_TIME_US) {
        return;
    }

    float dt = (now - lastTime) * 1e-6f;
    lastTime = now;

    // ===== READ IMU =====
    IMUData imu = imuUpdate(dt);

    // ===== SAFETY CHECK =====
    if (!isfinite(imu.roll) || !isfinite(imu.gyroRateX)
        || dt > 0.05f) {
        motorEnabled = false;
    }

    if (fabsf(imu.roll - targetRoll) > MAX_ANGLE) {
        motorEnabled = false;
    }

    // ===== SERIAL COMMAND =====
    while (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 'x' || cmd == 'X') {
            motorEnabled = false;
        }

        if (cmd == 's' || cmd == 'S') {
            if (isfinite(imu.roll) &&
                isfinite(imu.gyroRateX) &&
                fabsf(imu.roll - targetRoll) < ARM_ANGLE &&
                dt <= 0.05f) {

                motorEnabled = true;
            }
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

    // ===== DEBUG (20 Hz) =====
    if (millis() - lastPrint >= 50) {
        lastPrint = millis();

        Serial.printf(">Roll:%.2f\n", imu.roll);
        Serial.printf(">GyroRate:%.2f\n", imu.gyroRateX);
        Serial.printf(">PD_Output:%.2f\n", output);
        Serial.printf(">PWM_Actual:%d\n", pwm);
    }
}
