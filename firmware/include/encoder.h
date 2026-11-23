
#pragma once
#include <Arduino.h>

// Configure encoder pins and attach their interrupts.
void encoderInit();

// Read each pulse counter safely while its ISR may be updating it.
int32_t getLeftCount();
int32_t getRightCount();

// Reset both wheel pulse counters to zero.
void resetEncoder();

// Convert a pulse delta over dt seconds to RPM.
float calculateRPM(int32_t deltaCount, float dt);
