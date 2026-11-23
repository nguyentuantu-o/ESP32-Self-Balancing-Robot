#include <Arduino.h>

#include "config.h"
#include "encoder.h"
#include "speed_control.h"

namespace {
// The encoder loop runs at 20 Hz; gains below use RPM and wheel revolutions.
constexpr unsigned long SPEED_SAMPLE_US = 50000UL;
// First-order low-pass filter to reduce encoder noise and vibration.
constexpr float ERROR_FILTER_TIME_S = 0.15f;
constexpr float POSITION_KP = 40.0f; // Target RPM per wheel revolution of position error
constexpr float SPEED_KP = 0.03f;     // Degrees of roll offset per RPM of speed error
constexpr float SPEED_KI = 0.005f;    // Degrees of roll offset per (RPM * second)
constexpr float MAX_TARGET_RPM = 60.0f;
constexpr float MAX_TARGET_ROLL = 3.0f;
constexpr float MAX_SPEED_INTEGRAL = 200.0f;

// Previous encoder samples, estimated position, filter states, and PI integral.
unsigned long lastSpeedUs = 0;
int32_t previousLeftCount = 0;
int32_t previousRightCount = 0;
float positionRev = 0.0f;
float filteredPositionError = 0.0f;
float filteredSpeedError = 0.0f;
float speedIntegral = 0.0f;
// Latest output; held between encoder-loop updates.
SpeedData latestSpeed = {};
}

// Call after encoderInit() to establish the initial wheel-position reference.
void speedControlInit() {
    resetSpeedController();
}

void resetSpeedController() {
    // Rebase the counters so movement while disarmed does not create position error.
    previousLeftCount = getLeftCount();
    previousRightCount = getRightCount();
    lastSpeedUs = micros();
    positionRev = 0.0f;
    filteredPositionError = 0.0f;
    filteredSpeedError = 0.0f;
    speedIntegral = 0.0f;
    latestSpeed.leftRawRPM = 0.0f;
    latestSpeed.rightRawRPM = 0.0f;
    latestSpeed.speedRPM = 0.0f;
    latestSpeed.targetRPM = 0.0f;
    latestSpeed.positionErrorRev = 0.0f;
    latestSpeed.speedErrorRPM = 0.0f;
    latestSpeed.targetRoll = 0.0f;
}

// The encoder loop is slower than the balance loop, so update only once per sample period.
SpeedData updateSpeedLoop(unsigned long nowUs, bool enabled) {
    // Ignore movement while disarmed and continuously reset the position reference.
    if (!enabled) {
        resetSpeedController();
        return latestSpeed;
    }

    const unsigned long elapsedUs = nowUs - lastSpeedUs;
    if (elapsedUs < SPEED_SAMPLE_US) {
        return latestSpeed;
    }

    // Use the actual elapsed time between samples rather than assuming an exact period.
    const float dt = elapsedUs * 1e-6f;
    lastSpeedUs = nowUs;

    const int32_t leftCount = getLeftCount();
    const int32_t rightCount = getRightCount();

    // Unsigned subtraction handles int32 counter wraparound.
    const int32_t deltaLeft = static_cast<int32_t>(
        static_cast<uint32_t>(leftCount) -
        static_cast<uint32_t>(previousLeftCount)
    );
    const int32_t deltaRight = static_cast<int32_t>(
        static_cast<uint32_t>(rightCount) -
        static_cast<uint32_t>(previousRightCount)
    );
    previousLeftCount = leftCount;
    previousRightCount = rightCount;

    // Calculate each wheel's RPM before normalizing direction and averaging.
    latestSpeed.leftRawRPM = calculateRPM(deltaLeft, dt);
    latestSpeed.rightRawRPM = calculateRPM(deltaRight, dt);

    // Normalize forward direction, then average the two wheel speeds.
    const float forwardLeftRPM =
        LEFT_FORWARD_SIGN * latestSpeed.leftRawRPM;
    const float forwardRightRPM =
        RIGHT_FORWARD_SIGN * latestSpeed.rightRawRPM;
    const float measuredRPM = (forwardLeftRPM + forwardRightRPM) * 0.5f;

    // Integrate RPM into wheel revolutions, using the initial position as zero.
    positionRev += measuredRPM * dt / 60.0f;

    // Low-pass filter both the position error and the measured speed.
    const float filterAlpha = dt / (ERROR_FILTER_TIME_S + dt);
    const float positionError = -positionRev;
    filteredPositionError +=
        filterAlpha * (positionError - filteredPositionError);
    latestSpeed.positionErrorRev = filteredPositionError;

    // The position P loop generates a speed target; the speed PI loop generates a roll offset.
    latestSpeed.targetRPM = constrain(
        POSITION_KP * filteredPositionError,
        -MAX_TARGET_RPM,
        MAX_TARGET_RPM
    );

    // Filter measured speed before calculating the PI-loop error.
    latestSpeed.speedRPM +=
        filterAlpha * (measuredRPM - latestSpeed.speedRPM);

    // A forward-moving robot with a zero-position target produces a negative speed error.
    const float speedError = latestSpeed.targetRPM - latestSpeed.speedRPM;
    filteredSpeedError +=
        filterAlpha * (speedError - filteredSpeedError);
    latestSpeed.speedErrorRPM = filteredSpeedError;

    // Integrate speed error with a limit to prevent windup while the robot is constrained.
    const float proposedIntegral = constrain(
        speedIntegral + filteredSpeedError * dt,
        -MAX_SPEED_INTEGRAL,
        MAX_SPEED_INTEGRAL
    );
    const float unconstrainedTarget =
        SPEED_KP * filteredSpeedError + SPEED_KI * proposedIntegral;

    // Limit roll offset so the outer loop cannot request excessive lean.
    const float constrainedTarget = constrain(
        unconstrainedTarget,
        -MAX_TARGET_ROLL,
        MAX_TARGET_ROLL
    );

    // Integrate only when unsaturated or when the error will move the output out of saturation.
    if (constrainedTarget == unconstrainedTarget ||
        (constrainedTarget >= MAX_TARGET_ROLL && filteredSpeedError < 0.0f) ||
        (constrainedTarget <= -MAX_TARGET_ROLL && filteredSpeedError > 0.0f)) {
        speedIntegral = proposedIntegral;
    }

    latestSpeed.targetRoll = constrainedTarget;
    return latestSpeed;
}
