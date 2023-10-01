#pragma once
// Parameters for the Dragster run

static const int startFinishCountLimit = 1;
static const int steeringGainManual = 8;

static const int courseTimedDistance = 5625; //3780;          // 5625;
static const int courseTargetStoppingDistance = 400;  // 600;

// Marker thresholds
const int markerHighThreshold = (int)(0.12 * 4096); //0.92;
const int markerLowThreshold = (int)(0.10 * 4096); //0.88;

// Max valid line detectors before we abort the run
const int minLineDetectorThreshold = (int)(0.05 * 4096); //4000;

// Sample filtering to apply, to reduce noise, e.g. 3 applies a ratio 3 old readings to 1 new 
const int sampleFilteringFactor = 0;
// Number of samples averaged per sensor read cycle
const int sampleAveraging = 4;  

// Flags
const uint8_t FLAG_DISPLAY_LAST_RUN = 0x01;
const uint8_t FLAG_IGNORE_MARKERS = 0x02;
const uint8_t FLAG_COAST_ON_MAX_SPEED = 0x04;
const uint8_t FLAG_END_ON_MAX_SPEED = 0x08;
const uint8_t FLAG_END_ON_MIN_SPEED = 0x10;
const uint8_t FLAG_BREAK_STOP = 0x20;

// PID values
const float LOOP_INTERVAL = 0.004;  // Secs
const float LOOP_FREQUENCY = 1/LOOP_INTERVAL;  // Hz
const float MAX_MOTOR_VOLTS = 300.0;
// Defaults (actual from NVRam
const float PID_Kp = 0.03; //0.005; //0.01;// 0.007;
const float PID_Ki = 0.0;
const float PID_Kd = 0.005; //0.002;

typedef struct RunProfile
{
  const char *name;
  uint8_t flags;      // FLAG_ constants
  uint16_t runTimeMs; // Milliseconds
  int16_t startPower;
  int16_t endPower;   // So acceleration is (endPower - startPower)/runTimeMs/1000
  int16_t endDistance; // if not zero, end this segment if distance reading gets to this
  int16_t targetSpeed; // mm/S
  struct RunProfile *pNext;
  struct RunProfile *pStop;
} RunProfile_t;
