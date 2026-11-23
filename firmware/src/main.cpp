
#include <Arduino.h>
#include "config.h"
#include "imu.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "speed_control.h"

// Temporarily bypass motor control and report encoder direction for testing.
constexpr bool ENCODER_DIAGNOSTIC_MODE = false;

// Desired upright angle in degrees; the encoder loop adds an offset to this.
float targetRoll = 0.5f;

// Stop the motors beyond MAX_ANGLE; auto-arm is allowed only near upright.
const float MAX_ANGLE = 30.0f;
const float ARM_ANGLE = 5.0f;

// Timing references for the balance loop and Serial diagnostics.
unsigned long lastTime = 0;
unsigned long lastPrint = 0;
unsigned long lastEncoderDiagnostic = 0;
int32_t previousDiagnosticLeftCount = 0;
int32_t previousDiagnosticRightCount = 0;

// motorEnabled permits motor control; autoArm permits one automatic startup.
bool motorEnabled = false;
bool autoArm = true;

void setup() {
    // Start Serial first so startup errors and diagnostics can be reported.
    Serial.begin(115200);

    // A missing MPU blocks balancing, but encoder diagnostics can still run with motors stopped.
    if (!imuInit()) {
        Serial.println("MPU6050 ERROR!");
        if (!ENCODER_DIAGNOSTIC_MODE) {
            while (1) delay(100);
        }
    }

    // Initialize encoders and the outer loop, then configure and stop the motors.
    encoderInit();
    speedControlInit();
    motorInit();
    motorStop();

    // Capture the initial counts so diagnostics report only new wheel movement.
    previousDiagnosticLeftCount = getLeftCount();
    previousDiagnosticRightCount = getRightCount();
    lastEncoderDiagnostic = micros();
    if (ENCODER_DIAGNOSTIC_MODE) {
        autoArm = false;
    }

    // Set the initial timing reference for the periodic control loop.
    lastTime = micros();
}

void loop() {
    unsigned long now = micros();

    // In diagnostic mode, keep the motors stopped and print signed count deltas.
    if (ENCODER_DIAGNOSTIC_MODE) {
        if (now - lastEncoderDiagnostic >= 250000UL) {
            const int32_t leftCount = getLeftCount();
            const int32_t rightCount = getRightCount();
            const int32_t leftDelta = static_cast<int32_t>(
                static_cast<uint32_t>(leftCount) -
                static_cast<uint32_t>(previousDiagnosticLeftCount)
            );
            const int32_t rightDelta = static_cast<int32_t>(
                static_cast<uint32_t>(rightCount) -
                static_cast<uint32_t>(previousDiagnosticRightCount)
            );

            Serial.printf(
                "Encoder dCount | Left raw=%ld forward=%ld | "
                "Right raw=%ld forward=%ld\n",
                static_cast<long>(leftDelta),
                static_cast<long>(LEFT_FORWARD_SIGN * leftDelta),
                static_cast<long>(rightDelta),
                static_cast<long>(RIGHT_FORWARD_SIGN * rightDelta)
            );

            previousDiagnosticLeftCount = leftCount;
            previousDiagnosticRightCount = rightCount;
            lastEncoderDiagnostic = now;
        }
        return;
    }

    // Run the controller once per SAMPLE_TIME_US. Preserve the elapsed-period
    // remainder to reduce timing drift caused by work in previous iterations.
    unsigned long elapsed = now - lastTime;

    if (elapsed < SAMPLE_TIME_US) {
        return;
    }

    float dt = elapsed * 1e-6f;
    lastTime += (elapsed / SAMPLE_TIME_US) * SAMPLE_TIME_US;

    // Read the MPU and calculate roll/angular rate for the balance loop.
    IMUData imu = imuUpdate(dt);

    // Disable the motors and auto-arm if sensor data is invalid, the loop stalls,
    // or the robot has tilted beyond the safety threshold.
    if (!isfinite(imu.roll) ||
    !isfinite(imu.gyroRateX) ||
    dt > 0.05f ||
    fabsf(imu.roll - targetRoll) > MAX_ANGLE) {

    motorEnabled = false;
    autoArm = false;
}

    // After reset, arm the motors once only if sensor data is valid and the robot is upright.
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

    // While disarmed, clear controller integrators and reset the encoder position reference.
    if (!motorEnabled) {
        targetRoll = 0.0f;
        resetPID();
        resetSpeedController();
    }

    // Update the slower encoder loop; disable it while the motors are disarmed.
    const SpeedData speed = updateSpeedLoop(
        now,
        motorEnabled && SPEED_LOOP_ENABLED
    );

    // The final angle target is the upright reference plus the encoder-loop offset.
    const float commandedRoll = targetRoll +
        ((motorEnabled && SPEED_LOOP_ENABLED) ? speed.targetRoll : 0.0f);

    // Recheck the safety angle against the actual target, including the encoder offset.
    if (motorEnabled &&
        fabsf(imu.roll - commandedRoll) > MAX_ANGLE) {
        motorEnabled = false;
        autoArm = false;
        targetRoll = 0.0f;
        resetPID();
        resetSpeedController();
    }

    // Print diagnostics at 10 Hz to monitor the MPU and outer loop.
    // if (now - lastPrint >= 100000) {
    //     lastPrint = now;
    //     Serial.printf(
    //         "MPU Roll: %.2f deg | Speed: %.2f RPM | Pos err: %.3f rev | "
    //         "Speed err: %.2f RPM | Roll offset: %.2f deg\n",
    //         imu.roll,
    //         speed.speedRPM,
    //         speed.positionErrorRev,
    //         speed.speedErrorRPM,
    //         speed.targetRoll
    //     );
    // }

    // The inner PID uses the MPU angle and gyro rate to balance at commandedRoll.
    float output = 0.0f;

    if (motorEnabled) {
        output = calculatePD(
            commandedRoll,
            imu.roll,
            imu.gyroRateX
        );

        // Disable the motors if the PID output is invalid; never send invalid PWM.
        if (!isfinite(output)) {
            motorEnabled = false;
            targetRoll = 0.0f;
            resetPID();
            resetSpeedController();
            output = 0.0f;
        }
    }

    // Clamp the command, compensate for motor dead zone, and drive both wheels;
    // stop the motors whenever control is disabled.
 int pwm = 0;

if (motorEnabled) {
    output = constrain(output, -255.0f, 255.0f);

    pwm = compensateDeadzone(output);
    setMotor(pwm, pwm);
} else {
    motorStop();
}

}
