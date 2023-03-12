#pragma once
#include <EEPROM.h>

#pragma pack(push,1)

static const size_t NVRAM_DRAGSTER_NVRAM = 0;

typedef struct DRAGSTER_NVRAM_S 
{
  static const uint16_t nvramMagic = 0x2246;
  uint16_t magic; // Identifies if NVRAM data is valid
  int16_t steer_centre;
  float Kp;
  float Ki;
  float Kd;
} DRAGSTER_NVRAM_T;

static const size_t NVRAM_DRAGSTER_NVRAM_RACE_CONFIG = sizeof(DRAGSTER_NVRAM_T);

typedef struct DRAGSTER_NVRAM_RACE_CONFIG_S 
{
  static const uint16_t nvramMagic = 0x2247;
  uint16_t magic; // Identifies if NVRAM data is valid

  // These define the track itself
  int16_t timed_distance;
  int16_t target_stopping_distance;

  // Initial acceleration phase
  int16_t initial_power;
  int16_t acceleration_distance;
  int16_t max_power;
  int16_t target_top_speed;
  int16_t slowdown_distance_from_end;
  int16_t initial_deceleration_power;
  int16_t max_deceleration_power;
  int16_t target_end_speed;
  int16_t break_deceleration_power;
  
} DRAGSTER_NVRAM_RACE_CONFIG_T;


static const size_t NVRAM_MAX_SIZE = 256;

#pragma pack(pop)
