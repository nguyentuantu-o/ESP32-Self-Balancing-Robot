
#pragma once

void setPIDGains(float kp, float ki, float kd);
void getPIDGains(float &kp, float &ki, float &kd);
void resetPID();

float calculatePD(float targetRoll,
                  float currentRoll,
                  float gyroRate);
