#pragma once

#define SERIAL_DEBUG_PORT 
//#define LOG_RAW_SENSORS

static const float kp = 1.0;
static const float kd = 0.05;

static const float encode_calibrate_l = 0.123;// 0.132; // smaller = bigger turns
static const float encode_calibrate_r = 0.123;// 0.132;

// When motor goes in one direction, it tends to go faster than the other
static const float motor_compensation_left = 1.0;   // When forward, adjust the left motor by this amount only
static const float motor_compensation_right = 1.0;  // When in reverse, adjust the right motor by this amount only

static const int turn_angle_inertia_compensation = 8; // End the turns this much short, to allow for inertia taking it the rest of the way

static const float turning_diameter_mm = 39.0;  // Bigger means turns more

static const int forward_speed = 64;

// Sensor sensitivities defaults
static const int sensor_left_min_raw = 5000;
static const int sensor_left_max_raw = 30000;
static const int sensor_line_min_raw = 8200;
static const int sensor_line_max_raw = 65000;
static const int sensor_right_min_raw = 5000;
static const int sensor_right_max_raw = 30000;

#ifdef SERIAL_DEBUG_PORT
static UART &DebugPort = Serial1; // i.e. UART0 (pins 0&1)
#else
static UART &DebugPort = Serial;  // i.e. USB serial
#endif

static const int log_frequency = 1; // Number of sensor ticks, e.g. 2mS * 5 = 10mS
