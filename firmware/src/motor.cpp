
#include <Arduino.h>
#include "config.h"
#include "motor.h"

// ===== INITIALIZE MOTOR =====
void motorInit() {
    // Motor trai
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);

    // Motor phai
    pinMode(BIN1_PIN, OUTPUT);
    pinMode(BIN2_PIN, OUTPUT);

    // PWM ESP32 - Arduino core 2.x
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(PWMA_PIN, 0);
    ledcAttachPin(PWMB_PIN, 1);

    motorStop();
}

// ===== CONTROL MOTOR =====
void setMotor(int leftPWM, int rightPWM) {
    leftPWM = constrain(leftPWM, -PWM_MAX, PWM_MAX);
    rightPWM = constrain(rightPWM, -PWM_MAX, PWM_MAX);

    // ===== MOTOR TRAI =====
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

    // ===== MOTOR PHAI =====
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

    // ===== SET PWM =====
    ledcWrite(0, abs(leftPWM));
    ledcWrite(1, abs(rightPWM));
}

// ===== STOP MOTOR =====
void motorStop() {
    setMotor(0, 0);
}

// ===== TEST MOTOR =====
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

int compensateDeadzone(float output) {
    const int minPWM = 30;
    const float threshold = 1.0f;

    float magnitude = fabsf(output);

    // Neu output qua nho thi dung motor
    if (magnitude < threshold) {
        return 0;
    }

    // Gioi han output PID
    magnitude = constrain(magnitude, 0.0f, 255.0f);

    // Bu vung chet PWM
    int pwm = minPWM + (int)(
        (magnitude - threshold) * (255 - minPWM)
        / (255 - threshold)
    );

    pwm = constrain(pwm, 0, 255);

    // Giu nguyen chieu quay
    if (output < 0) {
        pwm = -pwm;
    }

    return pwm;
}
