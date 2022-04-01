// Parameters for the Dragster run

static const int MAX_SPEED_SLOW = 32;
static const int MAX_SPEED_MED = 56;
static const int MAX_SPEED_FAST = 80;
static const int MAX_SPEED_CRAZY = 128;

#define SPEED_A1 (maxRunSpeed / 4)
static const int TIME_SPEED_A1_mS = 200;
#define SPEED_A2 (maxRunSpeed / 2)
static const int TIME_SPEED_A2_mS = 400;
#define SPEED_MAX maxRunSpeed
static const int TIME_SPEED_MAX_mS = 1000;

#define SPEED_DECEL (maxRunSpeed / 4)
static const int TIME_SPEED_DECEL_mS = 2000;

static const int SPEED_FAST_STOP_REVSERSE = 12;
static const int TIME_FAST_STOP_mS = 1000;

static const int TIME_STOPPING_mS = 2000;

static const int startFinishCountLimit = 1;

// Marker thresholds
const int markerLowThreshold = 3400;
const int markerHighThreshold = 3600;

// Max valid line detectors before we abort the run
const int maxLineDetectorThreshold = 3800;
