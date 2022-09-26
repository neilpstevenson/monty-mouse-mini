#include <TFT_eSPI.h> // TTGO T-Display library

extern TFT_eSPI tft;
extern int value[6];
extern void plotPointers(void);
extern void plotLinear(const char *label, int x, int y);
