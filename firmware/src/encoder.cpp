
#include <Arduino.h>
#include "config.h"
#include "encoder.h"

// Bien dem encoder
volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;

// Bao ve bien dem giua ISR va loop
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// ===== MOTOR TRAI =====
void IRAM_ATTR leftEncoderISR() {
    bool b = digitalRead(ENCODER_A_B);

    portENTER_CRITICAL_ISR(&encoderMux);

    if (b == HIGH) {
        leftCount--;
    } else {
        leftCount++;
    }

    portEXIT_CRITICAL_ISR(&encoderMux);
}

// ===== MOTOR PHAI =====
void IRAM_ATTR rightEncoderISR() {
    bool b = digitalRead(ENCODER_B_B);

    portENTER_CRITICAL_ISR(&encoderMux);

    if (b == HIGH) {
        rightCount++;
    } else {
        rightCount--;
    }

    portEXIT_CRITICAL_ISR(&encoderMux);
}

// ===== INITIALIZE =====
void encoderInit() {
    pinMode(ENCODER_A_A, INPUT_PULLUP);
    pinMode(ENCODER_A_B, INPUT_PULLUP);

    pinMode(ENCODER_B_A, INPUT_PULLUP);
    pinMode(ENCODER_B_B, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A_A),
        leftEncoderISR,
        RISING
    );

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_B_A),
        rightEncoderISR,
        RISING
    );
}

// ===== READ COUNTS =====
int32_t getLeftCount() {
    portENTER_CRITICAL(&encoderMux);
    int32_t count = leftCount;
    portEXIT_CRITICAL(&encoderMux);

    return count;
}

int32_t getRightCount() {
    portENTER_CRITICAL(&encoderMux);
    int32_t count = rightCount;
    portEXIT_CRITICAL(&encoderMux);

    return count;
}

// ===== RESET =====
void resetEncoder() {
    portENTER_CRITICAL(&encoderMux);
    leftCount = 0;
    rightCount = 0;
    portEXIT_CRITICAL(&encoderMux);
}
constexpr float CPR = 697.0f;

float calculateRPM(int32_t deltaCount, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    return deltaCount * 60.0f / (CPR * dt);
}