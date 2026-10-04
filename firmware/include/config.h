#pragma once
#include <stdint.h>

// ESP32-WROVER-IE on DevKitC V4
constexpr int PIN_SDA = 13;
constexpr int PIN_SCL = 14;
constexpr int PIN_CAN_TX = 23;
constexpr int PIN_CAN_RX = 22;

constexpr uint8_t HOST_ID = 0x00;
constexpr uint8_t MOTOR_L_ID = 0x04;
constexpr uint8_t MOTOR_R_ID = 0x05;
constexpr float MOTOR_R_SIGN = -1.0f; // Right motor is mounted in mirror.

constexpr uint32_t LOOP_HZ = 200;
constexpr uint32_t LOOP_US = 1000000UL / LOOP_HZ;
constexpr uint32_t IMU_REPORT_US = 5000;
constexpr float KP_DEFAULT = 0.70f;
constexpr float KI_DEFAULT = 0.00f;
constexpr float KD_DEFAULT = 0.30f;
constexpr float I_TORQUE_LIMIT = 1.0f; // Nm
constexpr float TORQUE_LIMIT = 3.5f;   // Nm
constexpr float TILT_CUTOFF = 0.6f;    // rad, about 34 degrees
constexpr float ARM_MAX_PITCH = 0.20f; // rad, about 11 degrees
constexpr uint32_t IMU_TIMEOUT_MS = 50;
constexpr uint8_t CAN_TX_FAIL_LIMIT = 3;

constexpr float IMPACT_ANG_ACCEL = 40.0f;     // rad/s^2
constexpr uint8_t IMPACT_CONFIRM_TICKS = 3;   // 15 ms
constexpr uint16_t IMPACT_STALL_TICKS = 120;  // 600 ms
constexpr float IMPACT_STALL_RATE_MAX = 2.0f; // rad/s

constexpr uint32_t ZERO_WARMUP_MS = 1000;
constexpr uint32_t ZERO_SAMPLES = 200;
constexpr uint32_t ZERO_TIMEOUT_MS = 3000;
// Rear panel to upright: counterclockwise looking along the positive IMU axis
// (tail to tip), hence -90 degrees before applying TILT_SIGN. With sign -1,
// the offset added to the measured rear angle is +90 degrees.
constexpr float REAR_TO_UPRIGHT_RAD = -1.57079632679f;
constexpr int TILT_AXIS_X = 1;
constexpr int TILT_AXIS_Y = 2;
constexpr int TILT_AXIS_DEFAULT = TILT_AXIS_X;
constexpr int TILT_SIGN_DEFAULT = -1;

constexpr const char* WIFI_AP_SSID = "TM-Robot";
constexpr const char* WIFI_AP_PASS = "balance123";
