#ifndef UTIL_H
#define UTIL_H 1

#include "tuning.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int cmToEnc(double cm) {
  return cm / rotToCMRatio * encToRotRatio;
}

int degTurnToEnc(double deg) {
  return (wheelGap * 3.14159) / rotToCMRatio * encToRotRatio * (deg / 360);
}

#endif
