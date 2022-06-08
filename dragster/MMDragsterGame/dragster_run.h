// Parameters for the Dragster run

static const int MAX_SPEED_SLOW = 40;
static const int MAX_SPEED_MED = 52;
static const int MAX_SPEED_FAST = 60;
static const int MAX_SPEED_CRAZY = 128;

#define SPEED_A1 (maxRunSpeed / 4)
static const int TIME_SPEED_A1_mS = 200;
#define SPEED_A2 (maxRunSpeed / 2)
static const int TIME_SPEED_A2_mS = 200;
#define SPEED_MAX maxRunSpeed
static const int TIME_SPEED_MAX_mS = 1500;

#define SPEED_DECEL (maxRunSpeed / 4)
static const int TIME_SPEED_DECEL_mS = 2000;

static const int SPEED_FAST_STOP_REVSERSE = 32;
static const int TIME_FAST_STOP_mS = 800;

static const int TIME_STOPPING_mS = 4000;

static const int startFinishCountLimit = 2;

// Marker thresholds
const int markerHighThreshold = (int)(0.45 * 4096); //3400;
const int markerLowThreshold = (int)(0.35 * 4096); //2900;

// Max valid line detectors before we abort the run
const int maxLineDetectorThreshold = 4000;

typedef struct RunProfile
{
  uint8_t flags;      // TBD
  uint16_t runTimeMs; // Milliseconds
  int16_t startSpeed;
  int16_t endSpeed;   // So acceleration is (endSpeed - startSpeed)/runTimeMs/1000
  struct RunProfile *pNext;
  struct RunProfile *pStop;
} RunProfile_t;
