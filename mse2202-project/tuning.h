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
  {DRIVE, 26},
  {TURN, 95},
  {DRIVE, 33},
  {TURN, 90},
  {DRIVE, 45}
};

const bool cwNavigation = true;                           // True if the robot's navigation and turns are clockwise

// Robot Constants
const double wheelDiameter = 4.3;                         // Wheel's diameter from center to edge of rubber
const double wheelGap = 9.4;                              // Gap between wheels from center to center

const int encToRotRatio = 60;                             // Number of encoder ticks for one rotation of the wheel
const double rotToCMRatio = wheelDiameter * 3.14159;      // Ratio between rotations to centimeters using the wheel diameter

// General Tuning Constants
const int driveMaxPower = 255;                            // Maximum power of the drive
int brakePower = 25;                                      // Braking power (applied opposite of the direction of movement)
unsigned long brakeTime = 80;                             // Time to apply the braking power for

// Drive (Straight) Tuning Constants
const double drivekP = 6.5;                               // Constant of proportionality for driving forwards/backwards
const double drivekI = 3.5;                               // Constant of integration for driving forwards/backwards
const double driveSteerkP = -6.7;                         // Constant of proportionality for correcting the drive's steering to straight
const double driveAccelTime = 500;                        // Time to accelerate from 0 to 255 power

// Turn Tuning Constants
const double turnkP = 6.9;                                // Constant of proportionality for tank turning
const double turnkI = 15.5;                               // Constant of integration for tank turning
const double turnAccelTime = 300;                         // Time to accelerate from 0 to 255 power

#endif
