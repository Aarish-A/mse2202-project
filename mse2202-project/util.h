#ifndef UTIL_H
#define UTIL_H 1

#include "tuning.h"

// Return the sign of any value
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// Convert centimeters to the necessary encoder ticks
int cmToEnc(double cm) {
  return cm / rotToCMRatio * encToRotRatio;
}

// Convert angle to the necessary encoder ticks (for robot to tank turn to)
int degTurnToEnc(double deg) {
  return (wheelGap * 3.14159) / rotToCMRatio * encToRotRatio * (deg / 360);
}

#endif
