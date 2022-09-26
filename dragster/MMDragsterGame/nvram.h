#pragma once
#include <EEPROM.h>

#pragma pack(push,1)

typedef struct DRAGSTER_NVRAM_S 
{
  static const uint16_t nvramMagic = 0x2246;
  uint16_t magic; // Identifies if NVRAM data is valid
  int16_t steer_centre;
  float Kp;
  float Ki;
  float Kd;
} DRAGSTER_NVRAM_T;

#pragma pack(pop)
