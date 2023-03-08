#pragma once

static const float kp = 1.0;
static const float kd = 0.01;

static const float encode_calibrate_l = 0.132;
static const float encode_calibrate_r = 0.132;

static const float turning_diameter_mm = 39.0;  // Bigger means turns more

static const int forward_speed = 128;
static const int turn_leadin_speed = 64;
static const int turn_speed = 32;
static const int turn_leadout_speed = 64;
static const int turn_180_speed = 64;

// Distance parameters during wall follower
static const int wall_follow_left_distance = 90;
static const int wall_follow_forward_min_distance = 95;
static const int wall_follow_left_gap_threshold = 40;
static const int wall_follow_right_gap_threshold = 40;
static const int wall_follow_ahead_blocked_threshold = 75;

// Sensor sensitivities
static const int sensor_left_min_raw = 100;
static const int sensor_left_max_raw = 7000;
static const int sensor_frontleft_min_raw = 1000;
static const int sensor_frontleft_max_raw = 9000;
static const int sensor_frontright_min_raw = 1000;
static const int sensor_frontright_max_raw = 9000;
static const int sensor_right_min_raw = 100;
static const int sensor_right_max_raw = 7000;
