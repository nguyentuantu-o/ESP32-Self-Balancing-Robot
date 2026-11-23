
#include <Arduino.h>
#include <math.h>
#include "pid.h"

static float Kp = 16.0f;
static float Ki = 100.0f;
static float Kd = 0.8f;

static float integral = 0.0f;
static const float DT = 0.01f; // 100 Hz

void setPIDGains(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    integral = 0.0f;
}

void getPIDGains(float &kp, float &ki, float &kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void resetPID() {
    integral = 0.0f;
}

float calculatePD(float targetRoll,
                  float currentRoll,
                  float gyroRate) {

    float error = targetRoll - currentRoll;

    float P = Kp * error;

    integral += error * DT;

    float I = Ki * integral;

    // Gioi han thanh phan I de tranh tich luy qua lon
    I = constrain(I, -40.0f, 40.0f);

    if (Ki > 0.0f) {
        integral = I / Ki;
    } else {
        integral = 0.0f;
    }

    // D tinh truc tiep tu gyro, giam nhieu do dao ham
    float D = -Kd * gyroRate;

    float output = P + I + D;

    return constrain(output, -255.0f, 255.0f);
}
