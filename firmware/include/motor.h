
#pragma once

// Configure motor pins/PWM channels and stop the driver.
void motorInit();

// Set signed PWM for each wheel; the sign selects its rotation direction.
void setMotor(int leftPWM, int rightPWM);

// Stop both wheels.
void motorStop();

// Run a forward/reverse motor test; call only with the robot safely supported.
void motorTest();

// Compensate for the low-PWM dead zone while preserving the command direction.
int compensateDeadzone(float output);