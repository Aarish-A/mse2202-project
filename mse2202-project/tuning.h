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
  //  {DRIVE, 30},
  {TURN, 90}
  //  {DRIVE, 30},
  //  {TURN, enc90turn},
  //  {DRIVE, 25}
};

// Robot Constants
const double wheelDiameter = 4.3;
const double wheelGap = 9.4;

const int encToRotRatio = 60;
const double rotToCMRatio = wheelDiameter * 3.14159;

// General Tuning Constants
const int driveMaxPower = 255;
int brakePower = 20;
unsigned long brakeTime = 80;

// Drive (Straight) Tuning Constants
const double drivekP = 2.4;
const double drivekI = 0.5;
const double driveSteerkP = 3.9;
const double driveAccelTime = 700;

// Turn Tuning Constants
const bool cwNavigation = true;
const double turnkP = 2.5;
const double turnkI = 0.15;
const double turnAccelTime = 400;

#endif
