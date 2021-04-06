#ifndef TUNING_H
#define TUNING_H 1

enum driveState {
  STOP = 0,
  DRIVE,
  TURN,
  BRAKE
};

struct driveManeuver {
  driveState state;
  int target;
};

// Navigation Route
const driveManeuver driveManeuvers[] = {
  {DRIVE, 25},
  //  {TURN, enc90turn},
  //  {DRIVE, 30},
  //  {TURN, enc90turn},
  //  {DRIVE, 25}
};

// General Tuning Constants
const int driveMaxPower = 255;
int brakePower = 20;
unsigned long brakeTime = 80;

// Drive (Straight) Tuning Constants
const double drivekP = 2.8;
const double drivekI = 0.8;
const double driveSteerkP = 3.9;

const double driveAccelTime = 700;

// Turn Tuning Constants
const int encToRotRatio = 120;
const int rotToCMRatio = 4.2 * 3.14159;
const int enc90turn = 21;
const bool cwNavigation = true;

#endif
