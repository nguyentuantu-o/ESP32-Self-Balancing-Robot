
#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "pid.h"

// Default balance-loop gains; units depend on the roll and PWM scales.
static float Kp = 16.0f;
static float Ki = 100.0f;
static float Kd = 0.8f;

// Integral state and nominal balance-loop period.
static float integral = 0.0f;
static constexpr float DT = SAMPLE_TIME_US * 1e-6f;

// Update the gains and clear the old integral to avoid carrying prior error forward.
void setPIDGains(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    integral = 0.0f;
}

// Return the current gains to the caller.
void getPIDGains(float &kp, float &ki, float &kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

// Clear the integral when the motors stop or the controller needs to restart.
void resetPID() {
    integral = 0.0f;
}

// Calculate the motor command from angle error: P responds to error, I corrects
// persistent error, and D uses gyro rate directly to damp motion.
float calculatePD(float targetRoll,
                  float currentRoll,
                  float gyroRate) {

    // Positive error means the target angle is greater than the measured angle.
    float error = targetRoll - currentRoll;

    // P responds directly to the angle error.
    float P = Kp * error;

    // Accumulate error over time; clamp I to prevent integral windup.
    integral += error * DT;

    float I = Ki * integral;

    // Clamp the integral contribution to prevent it from growing too large.
    I = constrain(I, -40.0f, 40.0f);

    if (Ki > 0.0f) {
        integral = I / Ki;
    } else {
        integral = 0.0f;
    }

    // Apply D opposite the measured gyro rate to damp angular motion.
    float D = -Kd * gyroRate;

    // Sum all three terms and clamp the command to the valid PWM range.
    float output = P + I + D;

    return constrain(output, -255.0f, 255.0f);
}
