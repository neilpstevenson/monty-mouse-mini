#pragma once

//#define SERIAL_DEBUG_PORT 
#define LOG_RAW_SENSORS
#define MOTORS_USE_BREAK_MODE

static const float kp = 0.015; //0.009; //4.0;
static const float kd = 0.0015; //0.0002; //0.5;

static const float encode_calibrate_l = 0.123;// 0.132; // smaller = bigger turns
static const float encode_calibrate_r = 0.123;// 0.132;

// When motor goes in one direction, it tends to go faster than the other
static const float motor_compensation_left = 1.0;   // When forward, adjust the left motor by this amount only
static const float motor_compensation_right = 1.0;  // When in reverse, adjust the right motor by this amount only
static const int max_acceleration_per_update  = 15;

static const int turn_angle_inertia_compensation = 8; // End the turns this much short, to allow for inertia taking it the rest of the way

//static const float turning_diameter_mm = 39.0;  // Bigger means turns more

static const int forward_speed = 150;

// Sensor sensitivities defaults
static const int sensor_left_min_raw = 3500;
static const int sensor_left_max_raw = 19000;
static const int sensor_left_mid_min_raw = 7600;
static const int sensor_left_mid_max_raw = 38000;
static const int sensor_left_centre_min_raw = 5900;
static const int sensor_left_centre_max_raw = 29000;
static const int sensor_right_centre_min_raw = 8600;
static const int sensor_right_centre_max_raw = 30000;
static const int sensor_right_mid_min_raw = 9600;
static const int sensor_right_mid_max_raw = 30000;
static const int sensor_right_min_raw = 4200;
static const int sensor_right_max_raw = 16000;

#ifdef SERIAL_DEBUG_PORT
static UART &DebugPort = Serial1; // i.e. UART0 (pins 0&1)
#else
static UART &DebugPort = Serial;  // i.e. USB serial
#endif

static const int log_frequency = 5; // Number of sensor ticks, e.g. 2mS * 5 = 10mS
