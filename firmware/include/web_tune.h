
#pragma once

#include "imu.h"
#include "speed_control.h"

void webTuneInit();
void webTuneHandle();
bool webTuneTakeDisarmRequest();
void webTuneSetIMUData(
    const IMUData &data,
    float targetRoll,
    int pwm,
    const SpeedData &speed
);
