
#include <Arduino.h>
#include "config.h"
#include "motor.h"

// Configure driver pins and PWM channels, then stop the motors.
void motorInit() {
    // The driver's two IN pins select the left wheel direction.
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);

    // The driver's two IN pins select the right wheel direction.
    pinMode(BIN1_PIN, OUTPUT);
    pinMode(BIN2_PIN, OUTPUT);

    // Create two PWM channels and attach one to each motor enable pin.
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(PWMA_PIN, 0);
    ledcAttachPin(PWMB_PIN, 1);

    motorStop();
}

// Select direction from the PWM sign and speed from its absolute value.
void setMotor(int leftPWM, int rightPWM) {
    // Keep commands within the range supported by the driver.
    leftPWM = constrain(leftPWM, -PWM_MAX, PWM_MAX);
    rightPWM = constrain(rightPWM, -PWM_MAX, PWM_MAX);

    // Select the left wheel direction; drive both pins LOW when PWM is zero.
    if (leftPWM > 0) {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
    }
    else if (leftPWM < 0) {
        digitalWrite(AIN1_PIN, HIGH);
        digitalWrite(AIN2_PIN, LOW);
    }
    else {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, LOW);
    }

    // Select the right wheel direction; its wiring may differ from the left wheel.
    if (rightPWM > 0) {
        digitalWrite(BIN1_PIN, HIGH);
        digitalWrite(BIN2_PIN, LOW);
    }
    else if (rightPWM < 0) {
        digitalWrite(BIN1_PIN, LOW);
        digitalWrite(BIN2_PIN, HIGH);
    }
    else {
        digitalWrite(BIN1_PIN, LOW);
        digitalWrite(BIN2_PIN, LOW);
    }

    // PWM magnitude is unsigned; the IN pins already select the direction.
    ledcWrite(0, abs(leftPWM));
    ledcWrite(1, abs(rightPWM));
}

// Shared stop helper using the same signed-command convention as setMotor.
void motorStop() {
    setMotor(0, 0);
}

// Delayed motor test; do not call while the balance loop is running.
void motorTest() {
    Serial.println("Forward");
    setMotor(80, 80);
    delay(1000);

    motorStop();
    delay(2000);

    Serial.println("Backward");
    setMotor(-80, -80);
    delay(1000);

    motorStop();
    delay(2000);
}

// Compensate for the low-PWM dead zone while preserving the command direction.
int compensateDeadzone(float output) {
    // Tune each direction separately if the motor has asymmetric friction.
    const int minPWMForward = 40;
    const int minPWMBackward = 40;

    // Ignore commands below the threshold to avoid unnecessary motor jitter.
    const float threshold = 1.0f;

    // Never convert a non-finite value into a PWM command.
    if (!isfinite(output)) {
        return 0;
    }

    float magnitude = fabsf(output);

    // Stop the motor for controller output inside the dead zone.
    if (magnitude < threshold) {
        return 0;
    }

    magnitude = constrain(magnitude, 0.0f, 255.0f);

    int minPWM;

    if (output > 0.0f) {
        minPWM = minPWMForward;
    } else {
        minPWM = minPWMBackward;
    }

    // Map [threshold, 255] to [minPWM, 255] to overcome starting friction.
    float mappedPWM = minPWM
        + (magnitude - threshold)
        * (255.0f - minPWM)
        / (255.0f - threshold);

    int pwm = constrain((int)mappedPWM, 0, 255);

    // Restore the original sign so setMotor selects the correct direction.
    if (output < 0.0f) {
        pwm = -pwm;
    }

    return pwm;
}

