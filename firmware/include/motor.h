
#pragma once

void motorInit();
void setMotor(int leftPWM, int rightPWM);
void motorStop();
void motorTest();
int compensateDeadzone(float output);