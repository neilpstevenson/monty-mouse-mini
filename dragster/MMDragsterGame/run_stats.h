#pragma once

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
