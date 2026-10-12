
#pragma once
#include <Arduino.h>

void encoderInit();

int32_t getLeftCount();
int32_t getRightCount();

void resetEncoder();

float calculateRPM(int32_t deltaCount, float dt);
