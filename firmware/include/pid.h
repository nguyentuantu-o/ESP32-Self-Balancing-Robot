
#pragma once

// Update the balance-loop PID gains and clear the previous integral.
void setPIDGains(float kp, float ki, float kd);

// Read the current gains.
void getPIDGains(float &kp, float &ki, float &kd);

// Clear the integral state when the motors stop or a fault occurs.
void resetPID();

// Calculate the motor command from roll error and gyro angular rate.
float calculatePD(float targetRoll,
                  float currentRoll,
                  float gyroRate);
