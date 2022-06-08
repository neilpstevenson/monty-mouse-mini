#include <EEPROM.h>

#pragma pack(push,1)

typedef struct {
  uint16_t magic; // Identifies if NVRAM data is valid
  uint8_t steer_centre;
} DRAGSTER_NVRAM_T;

#pragma pack(pop)
