#pragma once
// Parameters for the Dragster run

static const int startFinishCountLimit = 1;
static const int steeringGainManual = 8;

static const int courseTimedDistance = 5625; //3780;          // 5625;
static const int courseTargetStoppingDistance = 400;  // 600;

// Marker thresholds
const int markerHighThreshold = (int)(0.85 * 4096); //0.92;
const int markerLowThreshold = (int)(0.80 * 4096); //0.88;

// Max valid line detectors before we abort the run
const int maxLineDetectorThreshold = (int)(0.99 * 4096); //4000;

// Flags
const uint8_t FLAG_DISPLAY_LAST_RUN = 0x01;
const uint8_t FLAG_IGNORE_MARKERS = 0x02;
const uint8_t FLAG_COAST_ON_MAX_SPEED = 0x04;
const uint8_t FLAG_END_ON_MAX_SPEED = 0x08;
const uint8_t FLAG_END_ON_MIN_SPEED = 0x10;
const uint8_t FLAG_BREAK_STOP = 0x20;

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
