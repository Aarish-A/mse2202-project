#ifndef TUNING_H
#define TUNING_H 1

// 1 is for actual navigation
// 2 is for tuning the drive (straight) algorithm
// 3 is for tuning the turning algorithm
#define OPERATION_MODE 1 

// General Tuning Constants
const int driveMaxPower = 255;
int brakePower = 50;
unsigned long brakeTime = 100;

// Drive (Straight) Tuning Constants
const double drivekP = 2.4;
const double drivekI = 0.05;
const double driveSteerkP = 8.9;

const double driveAccelTime = 800;

// Turn Tuning Constants
const int encToRotRatio = 120;
const int rotToCMRatio = 4.2 * 3.14159;
const int enc90turn = 21;
const bool cwNavigation = true;

#endif
