#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

enum class ControllerState : uint8_t { DISARMED, ARMING, RUNNING, FAULT };

extern ControllerState controller_state;
extern const char* fault_reason;
extern float kp_gain, ki_gain, kd_gain;
extern float pitch_raw, pitch_rate, pitch_bias;
extern bool calibrated;
extern float pitch_integral, last_i_tau, last_tau;
extern int tilt_axis, tilt_sign;
extern Preferences prefs;

// A few recent messages for the page to avoid ram related problems; continuous telemetry goes to Serial.
struct LogEntry { uint32_t ms; char text[96]; };
constexpr uint8_t LOG_CAPACITY = 8;
extern LogEntry log_entries[LOG_CAPACITY];
extern uint8_t log_write_index;

void log_message(const char* text);
const char* state_name();
float corrected_pitch();
void reset_integral();
const char* calibration_name();
bool calibration_active();
const char* arm_robot();
void stop_robot();
const char* reset_fault();
const char* start_calibration();
const char* set_tilt_axis(int axis, int sign);
void web_setup();
void web_loop();
