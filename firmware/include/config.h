
#pragma once

// I2C pins connecting the MPU6050 to the ESP32 and the sensor address.
#define MPU_SDA 21
#define MPU_SCL 22
#define MPU_ADDR 0x68

// The balance loop runs at 200 Hz; ALPHA weights the gyro in the complementary filter.
constexpr unsigned long SAMPLE_TIME_US = 5000; // 200 Hz
constexpr float ALPHA = 0.98f;

// Direction and PWM pins for the TB6612FNG motor driver.
#define AIN1_PIN 17
#define AIN2_PIN 16
#define PWMA_PIN 14

#define BIN1_PIN 5
#define BIN2_PIN 18
#define PWMB_PIN 33

// Encoder A/B channel pins for the two wheels.
#define ENCODER_A_A 19
#define ENCODER_A_B 23

#define ENCODER_B_A 32
#define ENCODER_B_B 27

// Sign corrections that make forward wheel rotation produce positive RPM.
constexpr int LEFT_FORWARD_SIGN = 1;
constexpr int RIGHT_FORWARD_SIGN = 1;

// Allow the encoder loop to adjust targetRoll while the motors are enabled.
constexpr bool SPEED_LOOP_ENABLED = false;

// PWM frequency, resolution, and maximum 8-bit command.
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define PWM_MAX 255
