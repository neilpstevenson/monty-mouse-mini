// Parameters for the Dragster run

static const int startFinishCountLimit = 1;
static const int steeringGainManual = 8;

static const int courseTimedDistance = 5625; //3780;          // 5625;
static const int courseTargetStoppingDistance = 400;  // 600;

// Marker thresholds
const int markerHighThreshold = (int)(0.6 * 4096); //0.92;
const int markerLowThreshold = (int)(0.5 * 4096); //0.88;

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


typedef struct Speeds
{ 
  static const int maxReadings = 24;
  struct {
    uint16_t timeMs;
    int16_t distanceMm;
  } readings[maxReadings];    // Index 0 is the oldest entry

  void logDistance(int16_t distanceMm)
  {
    // Copy all readings down one
    memcpy(readings, readings+1, sizeof(*readings) * (maxReadings-1));
    // Log new reading
    readings[maxReadings-1].timeMs = millis();
    readings[maxReadings-1].distanceMm = distanceMm;
 
  }
  
  int16_t getSpeed()
  {
    // Use first and last readings as speed assessment
    if(readings[0].timeMs)
      return (readings[maxReadings-1].distanceMm - readings[0].distanceMm) * 1000 / (readings[maxReadings-1].timeMs - readings[0].timeMs);
    else
      return 0;
  }
} Speeds_t;

typedef struct RunStats
{
  static const int maxStats = 16;
  int numStats;
  struct {
    uint16_t timeMs;
    int16_t distanceMm;
    int16_t speedMmS;
  } stats[maxStats];

  void clear()
  {
    numStats = 0;
  }
  
  void log(uint16_t timeMs,
          int16_t distanceMm,
          int16_t speedMmS)
  {
    if(numStats < maxStats)
    {
      stats[numStats].timeMs = timeMs;
      stats[numStats].distanceMm = distanceMm;
      stats[numStats].speedMmS = speedMmS;
      numStats++;
    }
  }
  
} RunStats_t;
