
#include "pid.h"

// Gia tri ban dau, chua tune
static float Kp = 7.0f;
static float Kd = 0.4f;

float calculatePD(float targetRoll,
                  float currentRoll,
                  float gyroRate) {

    float error = targetRoll - currentRoll;

    float P = Kp * error;
    float D = -Kd * gyroRate;

    return P + D;
}
