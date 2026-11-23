
#include <Arduino.h>
#include "config.h"
#include "encoder.h"

// ISR-updated pulse counters; volatile forces each access to read the latest value.
volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;

// Shared lock keeps counter reads/updates atomic between the ISRs and main loop.
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// Left-wheel channel-A ISR: read channel B to determine direction on each rising edge.
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

// Right-wheel channel-A ISR; RIGHT_FORWARD_SIGN normalizes its direction convention.
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

// Configure encoder inputs with pull-ups and count rising edges on channel A only.
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

// Snapshot the left counter while preventing an ISR from changing it mid-read.
int32_t getLeftCount() {
    portENTER_CRITICAL(&encoderMux);
    int32_t count = leftCount;
    portEXIT_CRITICAL(&encoderMux);

    return count;
}

// Snapshot the right counter using the same atomic-read pattern.
int32_t getRightCount() {
    portENTER_CRITICAL(&encoderMux);
    int32_t count = rightCount;
    portEXIT_CRITICAL(&encoderMux);

    return count;
}

// Reset both counters atomically so an ISR cannot split the two assignments.
void resetEncoder() {
    portENTER_CRITICAL(&encoderMux);
    leftCount = 0;
    rightCount = 0;
    portEXIT_CRITICAL(&encoderMux);
}

// Encoder counts per wheel revolution for the current hardware configuration.
constexpr float CPR = 697.0f;

// Convert a pulse delta over dt seconds to RPM; return zero for invalid dt.
float calculateRPM(int32_t deltaCount, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    return deltaCount * 60.0f / (CPR * dt);
}