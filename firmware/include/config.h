
#pragma once

// ===== MPU6050 =====
#define MPU_SDA 21
#define MPU_SCL 22
#define MPU_ADDR 0x68

// ===== CONTROL LOOP =====
constexpr unsigned long SAMPLE_TIME_US = 10000; // 100 Hz
constexpr float ALPHA = 0.98f;

// ===== MOTOR TB6612FNG =====
#define AIN1_PIN 17
#define AIN2_PIN 16
#define PWMA_PIN 14

#define BIN1_PIN 5
#define BIN2_PIN 18
#define PWMB_PIN 33

// ===== ENCODER =====
#define ENCODER_A_A 19
#define ENCODER_A_B 23

#define ENCODER_B_A 32
#define ENCODER_B_B 27

// ===== PWM =====
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define PWM_MAX 255
