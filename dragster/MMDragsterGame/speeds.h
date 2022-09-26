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
