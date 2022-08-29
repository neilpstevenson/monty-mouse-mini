// Parameters for the Dragster run

static const int startFinishCountLimit = 1;
static const int steeringGainManual = 8;

static const int courseTimedDistance = 3780;          // 5625;
static const int courseTargetStoppingDistance = 400;  // 600;

static const int speedAssessmentTime = 50; // Interval overwhich speed is assessed, in mS

// Marker thresholds
const int markerHighThreshold = (int)(0.92 * 4096); //3400;
const int markerLowThreshold = (int)(0.88 * 4096); //2900;

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
  int16_t targetSpeed; // set flags to determine behaviour
  struct RunProfile *pNext;
  struct RunProfile *pStop;
} RunProfile_t;

typedef enum
{
  STATE_INITIAL,
  STATE_MANUAL,
  STATE_PROFILE_DISARMED,
  STATE_PROFILE_ARMED,
  STATE_PROFILE_RUN,
  STATE_PROFILE_STOPPING,
  STATE_STEER_CALIBRATE,
  STATE_PID_CALIBRATE,
  STATE_POSITION_CALIBRATE
} EStates;
EStates state;
