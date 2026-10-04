#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include "driver/twai.h"
#include "common.h"

ControllerState controller_state = ControllerState::DISARMED;
const char* fault_reason = "none";
float kp_gain = KP_DEFAULT, ki_gain = KI_DEFAULT, kd_gain = KD_DEFAULT;
float pitch_raw = 0, pitch_rate = 0, pitch_bias = 0;
bool calibrated = false;
float pitch_integral = 0, last_i_tau = 0, last_tau = 0;
int tilt_axis = TILT_AXIS_DEFAULT, tilt_sign = TILT_SIGN_DEFAULT;
Preferences prefs;
LogEntry log_entries[LOG_CAPACITY] = {};
uint8_t log_write_index = 0;
static bool can_bus_off = false;
static uint8_t can_tx_failures = 0;

void log_message(const char* text) {
    Serial.println(text);
    LogEntry& entry = log_entries[log_write_index];
    entry.ms = millis();
    snprintf(entry.text, sizeof(entry.text), "%s", text);
    log_write_index = (log_write_index + 1) % LOG_CAPACITY;
}

// CyberGear uses extended 29-bit CAN IDs at 1 Mbit/s:
//   [28:24] command type, [23:8] command data, [7:0] target motor ID.
// Type 1: torque (-12..12 Nm) is mapped to an unsigned 16-bit value in
// the ID. Its 8 data bytes are four big-endian uint16 values:
// position (-4pi..4pi rad), velocity (-30..30 rad/s), Kp (0..500), Kd (0..5).
// We request pure torque: position, velocity and both motor gains are zero.
// Zero position/velocity encode as 0x7FFF, so the payload below is not all zero.
// Types 3 (enable), 4 (stop), 18 (parameter write) put the host ID in [15:8].
// Enable/stop use eight zero bytes. For type 18, bytes 0/1 contain the
// little-endian parameter index and byte 4 holds a uint8 parameter value.
// Selecting run_mode 0 means writing index 0x7005 with value 0 before enable.
static uint16_t f_to_u16(float value, float minimum, float maximum) {
    if (value < minimum) value = minimum;
    if (value > maximum) value = maximum;
    return (uint16_t)((value - minimum) / (maximum - minimum) * 65535.0f);
}

static bool send_can(uint8_t type, uint16_t data, uint8_t motor, const uint8_t payload[8]) {
    if (can_bus_off) return false;
    twai_message_t frame = {};
    frame.extd = 1;
    frame.identifier = ((uint32_t)type << 24) | ((uint32_t)data << 8) | motor;
    frame.data_length_code = 8;
    memcpy(frame.data, payload, 8);
    bool ok = twai_transmit(&frame, pdMS_TO_TICKS(5)) == ESP_OK;
    // Success means the driver accepted the frame, not that the motor enabled.
    if (ok) can_tx_failures = 0;
    else if (can_tx_failures < 255) ++can_tx_failures;
    return ok;
}

static bool motor_torque(uint8_t motor, float torque) {
    if (torque > TORQUE_LIMIT) torque = TORQUE_LIMIT;
    if (torque < -TORQUE_LIMIT) torque = -TORQUE_LIMIT;
    const uint8_t payload[8] = {0x7F, 0xFF, 0x7F, 0xFF, 0, 0, 0, 0};
    return send_can(1, f_to_u16(torque, -12.0f, 12.0f), motor, payload);
}

static bool motor_stop(uint8_t motor) {
    const uint8_t payload[8] = {};
    return send_can(4, HOST_ID, motor, payload);
}

static bool motor_enable(uint8_t motor) {
    const uint8_t zero[8] = {};
    const uint8_t mode[8] = {0x05, 0x70, 0, 0, 0, 0, 0, 0};
    if (!motor_stop(motor)) return false;
    delay(5);
    if (!send_can(18, HOST_ID, motor, mode)) return false;
    delay(5);
    if (!send_can(3, HOST_ID, motor, zero)) return false;
    delay(5);
    return true;
}

void reset_integral() {
    pitch_integral = 0;
    last_i_tau = 0;
}

static void stop_motors() {
    if (can_bus_off) return; // No CAN frame can be delivered in this state.
    motor_torque(MOTOR_L_ID, 0);
    motor_torque(MOTOR_R_ID, 0);
    delay(2);
    motor_stop(MOTOR_L_ID);
    motor_stop(MOTOR_R_ID);
}

static void enter_fault(const char* reason) {
    bool first_fault = controller_state != ControllerState::FAULT;
    controller_state = ControllerState::FAULT;
    fault_reason = reason;
    reset_integral();
    if (first_fault) log_message(reason);
    stop_motors();
}

static void poll_can() {
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK && (alerts & TWAI_ALERT_BUS_OFF)) {
        can_bus_off = true;
        enter_fault("CAN bus-off: reboot required");
    }
    // Consume replies so the RX queue cannot fill. They are not used by the PID.
    for (uint8_t i = 0; i < 32; ++i) {
        twai_message_t frame;
        if (twai_receive(&frame, 0) != ESP_OK) break;
    }
}

static Adafruit_BNO08x imu(-1);
static sh2_SensorValue_t imu_event;
static uint32_t last_orientation_ms = 0, last_gyro_ms = 0;
static uint32_t orientation_sequence = 0;
static bool calibration_imu_reset = false;

static bool enable_imu_reports() {
    bool orientation = imu.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_US);
    bool gyro = imu.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_REPORT_US);
    return orientation && gyro;
}

static float quat_to_tilt(float w, float x, float y, float z) {
    // Gravity projected onto the selected tilt plane. atan2 keeps the angle
    // continuous through the rear-panel pose, including Y beyond +/-90 degrees.
    float norm = w*w + x*x + y*y + z*z;
    if (!isfinite(norm) || norm <= 0) return NAN;
    if (tilt_axis == TILT_AXIS_X)
        return atan2f(2.0f * (w*x + y*z), norm - 2.0f * (x*x + y*y));
    return atan2f(2.0f * (w*y - x*z), norm - 2.0f * (x*x + y*y));
}

static void poll_imu() {
    if (imu.wasReset()) {
        calibration_imu_reset = calibration_active();
        last_orientation_ms = last_gyro_ms = 0;
        log_message("IMU reset: enabling reports");
        enable_imu_reports();
    }
    // Adafruit handles the SHTP packets. Use the quaternion for angle and
    // the calibrated gyro for the derivative, with the same axis and sign.
    while (imu.getSensorEvent(&imu_event)) {
        if (imu_event.sensorId == SH2_GAME_ROTATION_VECTOR) {
            const auto& q = imu_event.un.gameRotationVector;
            float angle = tilt_sign * quat_to_tilt(q.real, q.i, q.j, q.k);
            if (isfinite(angle)) {
                pitch_raw = angle;
                last_orientation_ms = millis();
                ++orientation_sequence;
            }
        } else if (imu_event.sensorId == SH2_GYROSCOPE_CALIBRATED) {
            const auto& g = imu_event.un.gyroscope;
            float rate = tilt_sign * (tilt_axis == TILT_AXIS_X ? g.x : g.y);
            if (isfinite(rate)) {
                pitch_rate = rate;
                last_gyro_ms = millis();
            }
        }
    }
}

static bool imu_fresh(uint32_t now) {
    // Each stream needs its own timestamp: one must not hide the other's loss.
    return last_orientation_ms != 0 && last_gyro_ms != 0 &&
           now - last_orientation_ms <= IMU_TIMEOUT_MS &&
           now - last_gyro_ms <= IMU_TIMEOUT_MS;
}

static float wrap_angle(float angle) {
    constexpr float pi = 3.14159265359f;
    while (angle > pi) angle -= 2.0f * pi;
    while (angle < -pi) angle += 2.0f * pi;
    return angle;
}

float corrected_pitch() { return wrap_angle(pitch_raw - pitch_bias); }

enum class CalibrationState : uint8_t { IDLE, WARMUP, SAMPLING };
static CalibrationState calibration_state = CalibrationState::IDLE;
static uint32_t calibration_phase_ms = 0, calibration_last_sequence = 0;
static uint32_t calibration_samples = 0;
static double calibration_sin = 0, calibration_cos = 0;

bool calibration_active() { return calibration_state != CalibrationState::IDLE; }

const char* calibration_name() {
    if (calibration_state == CalibrationState::WARMUP) return "warmup";
    if (calibration_state == CalibrationState::SAMPLING) return "sampling";
    return "idle";
}

const char* start_calibration() {
    if (controller_state != ControllerState::DISARMED || calibration_active())
        return "Calibration requires DISARMED and idle";
    calibration_state = CalibrationState::WARMUP;
    calibration_phase_ms = millis();
    calibration_last_sequence = orientation_sequence;
    calibration_samples = 0;
    calibration_sin = calibration_cos = 0;
    calibration_imu_reset = false;
    reset_integral();
    log_message("Rest motionless on rear panel on a level surface: calibrating");
    return nullptr;
}

static void process_calibration() {
    uint32_t now = millis();
    if (calibration_active() && calibration_imu_reset) {
        calibration_state = CalibrationState::IDLE;
        log_message("Calibration failed: IMU reset");
        return;
    }
    if (calibration_state == CalibrationState::WARMUP) {
        if (now - calibration_phase_ms >= ZERO_WARMUP_MS) {
            calibration_state = CalibrationState::SAMPLING;
            calibration_phase_ms = now;
            calibration_last_sequence = orientation_sequence;
            calibration_samples = 0;
            calibration_sin = calibration_cos = 0;
        }
    } else if (calibration_state == CalibrationState::SAMPLING) {
        if (imu_fresh(now) && orientation_sequence != calibration_last_sequence) {
            if (fabsf(pitch_rate) > 0.05f) {
                calibration_state = CalibrationState::IDLE;
                log_message("Calibration failed: keep the robot still");
                return;
            }
            // Circular mean also works when samples cross -pi/+pi.
            calibration_sin += sinf(pitch_raw);
            calibration_cos += cosf(pitch_raw);
            ++calibration_samples;
            calibration_last_sequence = orientation_sequence;
        }
        if (calibration_samples >= ZERO_SAMPLES) {
            float rear = tilt_sign * (float)atan2(calibration_sin, calibration_cos);
            pitch_bias = wrap_angle(tilt_sign * (rear + REAR_TO_UPRIGHT_RAD));
            calibrated = true;
            prefs.putFloat("rear", rear);
            prefs.putUChar("rear_axis", tilt_axis);
            calibration_state = CalibrationState::IDLE;
            reset_integral();
            char message[96];
            snprintf(message, sizeof(message), "Rear reference: rear=%+.4f offset=%+.4f upright=%+.4f rad",
                     tilt_sign * rear, tilt_sign * REAR_TO_UPRIGHT_RAD, pitch_bias);
            log_message(message);
            log_message("Calibration saved: stand upright before arming");
        } else if (now - calibration_phase_ms >= ZERO_TIMEOUT_MS) {
            calibration_state = CalibrationState::IDLE;
            log_message("Calibration failed: not enough fresh IMU samples");
        }
    }
}

const char* state_name() {
    switch (controller_state) {
        case ControllerState::DISARMED: return "DISARMED";
        case ControllerState::ARMING: return "ARMING";
        case ControllerState::RUNNING: return "RUNNING";
        case ControllerState::FAULT: return "FAULT";
    }
    return "unknown";
}

const char* arm_robot() {
    if (controller_state != ControllerState::DISARMED || calibration_active())
        return "Arm requires DISARMED and idle";
    if (can_bus_off) return "CAN bus-off: reboot required";
    if (!calibrated) return "Calibrate on the rear panel before arming";
    if (!imu_fresh(millis())) return "IMU data stale";
    if (fabsf(corrected_pitch()) > ARM_MAX_PITCH) return "Hold the robot upright before arming";
    controller_state = ControllerState::ARMING;
    fault_reason = "none";
    can_tx_failures = 0;
    reset_integral();
    log_message("Arming motors");
    if (!motor_enable(MOTOR_L_ID) || !motor_enable(MOTOR_R_ID)) {
        enter_fault("CAN transmission failed");
        return fault_reason;
    }
    controller_state = ControllerState::RUNNING;
    log_message("RUNNING");
    return nullptr;
}

void stop_robot() {
    stop_motors();
    reset_integral();
    if (controller_state != ControllerState::FAULT) {
        controller_state = ControllerState::DISARMED;
        fault_reason = "none";
    }
    log_message(can_bus_off ? "CAN bus-off: STOP cannot be delivered" : "Stop requested");
}

const char* reset_fault() {
    if (can_bus_off) return "CAN bus-off: reboot required";
    if (controller_state != ControllerState::FAULT) return "No fault to reset";
    if (!imu_fresh(millis())) return "IMU data stale";
    if (fabsf(corrected_pitch()) > ARM_MAX_PITCH) return "Hold the robot upright before reset";
    controller_state = ControllerState::DISARMED;
    fault_reason = "none";
    can_tx_failures = 0;
    reset_integral();
    log_message("Fault reset: DISARMED");
    return nullptr;
}

const char* set_tilt_axis(int axis, int sign) {
    if (controller_state != ControllerState::DISARMED || calibration_active())
        return "Mounting changes require DISARMED and idle";
    if ((axis != TILT_AXIS_X && axis != TILT_AXIS_Y) || (sign != 1 && sign != -1))
        return "Invalid tilt axis or sign";
    if (axis != tilt_axis || sign != tilt_sign) {
        tilt_axis = axis;
        tilt_sign = sign;
        pitch_bias = 0;
        calibrated = false;
        prefs.putUChar("rear_axis", 0);
        reset_integral();
        log_message("IMU orientation changed: recalibrate on rear panel");
    }
    return nullptr;
}

static void serial_commands() {
    while (Serial.available()) {
        const char* error = nullptr;
        switch (Serial.read()) {
            case 'a': case 'A': error = arm_robot(); break;
            case 's': case 'S': stop_robot(); break;
            case 'r': case 'R': error = reset_fault(); break;
            case 'c': case 'C': error = start_calibration(); break;
        }
        if (error) log_message(error);
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    prefs.begin("tmrobot", false);
    kp_gain = prefs.isKey("kp") ? prefs.getFloat("kp", KP_DEFAULT) : KP_DEFAULT;
    ki_gain = prefs.isKey("ki") ? prefs.getFloat("ki", KI_DEFAULT) : KI_DEFAULT;
    kd_gain = prefs.isKey("kd") ? prefs.getFloat("kd", KD_DEFAULT) : KD_DEFAULT;
    tilt_axis = prefs.getUChar("axis", TILT_AXIS_DEFAULT);
    tilt_sign = prefs.getChar("sign", TILT_SIGN_DEFAULT);
    if (tilt_axis != TILT_AXIS_X && tilt_axis != TILT_AXIS_Y) tilt_axis = TILT_AXIS_DEFAULT;
    if (tilt_sign != 1 && tilt_sign != -1) tilt_sign = TILT_SIGN_DEFAULT;
    float rear = prefs.getFloat("rear", NAN);
    calibrated = isfinite(rear) && fabsf(rear) <= 3.14159265359f &&
                 prefs.getUChar("rear_axis", 0) == tilt_axis;
    if (calibrated) pitch_bias = wrap_angle(tilt_sign * (rear + REAR_TO_UPRIGHT_RAD));

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    if (!imu.begin_I2C() || !enable_imu_reports()) {
        log_message("BNO085 initialization failed");
        while (true) delay(100);
    }
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
    g.tx_queue_len = 32;
    g.rx_queue_len = 32;
    g.alerts_enabled = TWAI_ALERT_BUS_OFF;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
        log_message("CAN initialization failed");
        while (true) delay(100);
    }
    web_setup();
    stop_motors();
    log_message(calibrated ? "Rear-panel reference loaded" : "Calibrate on rear panel before arming");
    log_message("DISARMED: a=arm s=stop r=reset c=calibrate");
}

void loop() {
    static uint32_t last_tick = 0, ticks = 0;
    static bool send_left = true;
    static float previous_rate = 0;
    static uint8_t impact_ticks = 0;
    static uint16_t stall_ticks = 0;
    web_loop();
    serial_commands();
    uint32_t now = micros();
    if (now - last_tick < LOOP_US) return;
    last_tick = now;

    poll_imu();
    process_calibration();
    poll_can();
    float pitch = corrected_pitch();
    float torque = 0;
    if (controller_state == ControllerState::RUNNING) {
        if (!imu_fresh(millis())) enter_fault("IMU data stale");
        else if (fabsf(pitch) > TILT_CUTOFF) enter_fault("Tilt cutoff");
        else {
            if (ki_gain > 0) {
                pitch_integral += pitch * (1.0f / (float)LOOP_HZ);
                float limit = I_TORQUE_LIMIT / ki_gain;
                if (pitch_integral > limit) pitch_integral = limit;
                if (pitch_integral < -limit) pitch_integral = -limit;
                last_i_tau = ki_gain * pitch_integral;
            } else reset_integral();
            torque = kp_gain * pitch + last_i_tau + kd_gain * pitch_rate;
            if (torque > TORQUE_LIMIT) torque = TORQUE_LIMIT;
            if (torque < -TORQUE_LIMIT) torque = -TORQUE_LIMIT;

            // A sudden jolt needs three consecutive high-acceleration ticks.
            float acceleration = (pitch_rate - previous_rate) * (float)LOOP_HZ;
            previous_rate = pitch_rate;
            if (fabsf(acceleration) > IMPACT_ANG_ACCEL) {
                if (++impact_ticks >= IMPACT_CONFIRM_TICKS) {
                    enter_fault("Impact detected");
                    impact_ticks = 0;
                }
            } else impact_ticks = 0;
            // Sustained saturation with little rotation can indicate a blocked robot.
            if (fabsf(torque) >= TORQUE_LIMIT - 0.05f && fabsf(pitch_rate) < IMPACT_STALL_RATE_MAX) {
                if (++stall_ticks >= IMPACT_STALL_TICKS) {
                    enter_fault("Impact detected");
                    stall_ticks = 0;
                }
            } else stall_ticks = 0;

            if (controller_state == ControllerState::RUNNING) {
                // One frame per tick: each motor receives commands at 100 Hz.
                bool ok = send_left ? motor_torque(MOTOR_L_ID, torque)
                                    : motor_torque(MOTOR_R_ID, MOTOR_R_SIGN * torque);
                if (!ok && can_tx_failures >= CAN_TX_FAIL_LIMIT)
                    enter_fault("CAN transmission failed");
                send_left = !send_left;
            }
        }
    } else {
        reset_integral();
        previous_rate = pitch_rate;
        impact_ticks = 0;
        stall_ticks = 0;
    }
    last_tau = torque;
    uint32_t divisor = controller_state == ControllerState::RUNNING ? 10 : 100;
    if (++ticks % divisor == 0)
        Serial.printf("%s pitch=%+.3f rate=%+.3f torque=%+.2f I=%+.2f\n",
                      state_name(), pitch, pitch_rate, torque, last_i_tau);
}
