#ifndef UTIL_H
#define UTIL_H 1

#include "tuning.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int cmToEnc(int cm) {
  return cm / rotToCMRatio * encToRotRatio;
}

#endif
