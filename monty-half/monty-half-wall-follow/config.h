#pragma once

#define SERIAL_DEBUG_PORT 
#define LOG_RAW_SENSORS

static const float kp = 1.0;
static const float kd = 0.01;

static const float encode_calibrate_l = 0.123;// 0.132; // smaller = bigger turns
static const float encode_calibrate_r = 0.123;// 0.132;

static const float motor_compensation_left = 0.79;  // When motor goes in one direction, it tends to go faster than the other
static const float motor_compensation_right = 1.12;

static const int turn_angle_inertia_compensation = 8; // End the turns this much short, to allow for inertia taking it the rest of the way

static const float turning_diameter_mm = 39.0;  // Bigger means turns more

static const float wall_sensor_filter_ratio = 0.5;  // average = old * (1.0-ratio) + new * ratio

static const int forward_speed = 128;
static const int turn_leadin_speed = 64;
static const int turn_speed = 32;
static const int turn_leadout_speed = 64;
static const int turn_180_speed = 64;

// Distance parameters during wall follower
static const int wall_follow_left_distance = 85; //90;
static const int wall_follow_left_distance_min = 50;
static const int wall_follow_forward_min_distance = 98; //90; //95;
static const int wall_follow_left_gap_threshold = 40;
static const int wall_follow_right_gap_threshold = 40;
static const int wall_follow_ahead_blocked_threshold = 40;

// Distance parameters during simple wall follower
static const int wall_follow_simple_left_gap_threshold = 40;
static const int wall_follow_simple_right_gap_threshold = 40;
static const int wall_follow_simple_ahead_blocked_threshold = 95;
static const int wall_follower_simple_left_turn_delay = 50; // In loops

// Distances to move during various manoevres
static const int wall_follow_move_left_initial_forward = 85;

// Sensor sensitivities
static const int sensor_left_min_raw = 7000;
static const int sensor_left_max_raw = 12000;
static const int sensor_frontleft_min_raw = 8000;
static const int sensor_frontleft_max_raw = 55000;
static const int sensor_frontright_min_raw = 8000;
static const int sensor_frontright_max_raw = 55000;
static const int sensor_right_min_raw = 7000;
static const int sensor_right_max_raw = 12000;

#ifdef SERIAL_DEBUG_PORT
static UART &DebugPort = Serial1; // i.e. UART0 (pins 0&1)
#else
static UART &DebugPort = Serial;  // i.e. USB serial
#endif

