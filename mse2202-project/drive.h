#ifndef DRIVE_H
#define DRIVE_H 1

#include "tuning.h"
#include "util.h"

// Global Movement Measuring Variables
const int printTime = 1500;

int error1 = 0;
int error2 = 0;
int power1 = 0;
int power2 = 0;

int proportional1 = 0;
int proportional2 = 0;
double integral = 0;

// Structures and Enums
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

const driveManeuver driveManeuvers[] = {
  {DRIVE, 25},
//  {TURN, enc90turn},
//  {DRIVE, 30},
//  {TURN, enc90turn},
//  {DRIVE, 25}
};

const int nDriveManeuvers = sizeof(driveManeuvers) / sizeof(driveManeuver);

// Drive State and Maneuver Management Variables
unsigned long driveStateTime = 0;
driveState curDriveState = STOP;
driveState prevDriveState = STOP;
unsigned int driveManeuverIndex = 0;

void resetMeasurements() {
  ENC_ClearOdometer();
  error1 = 0;
  error2 = 0;
  power1 = 0;
  power2 = 0;

  proportional1 = 0;
  proportional2 = 0;
  integral = 0;
}

void changeState(driveState nextState) {
  prevDriveState = curDriveState;
  curDriveState = nextState;

  switch (curDriveState) {
    STOP:
      Serial.printf("Switched state to STOP, took %lu time", millis() - driveStateTime);
      driveManeuverIndex = 0;
      break;
    DRIVE:
      Serial.printf("Switched state to DRIVE, took %lu time", millis() - driveStateTime);
      resetMeasurements();
      break;
    TURN:
      Serial.printf("Switched state to TURN, took %lu time", millis() - driveStateTime);
      resetMeasurements();
      break;
    BRAKE:
      Serial.printf("Switched state to BRAKE, took %lu time", millis() - driveStateTime);
      break;
  }

  driveStateTime = millis();
}

void setupDrive() {
  ledcAttachPin(ciMotorLeftA, 1); // assign Motors pins to channels
  ledcAttachPin(ciMotorLeftB, 2);
  ledcAttachPin(ciMotorRightA, 3);
  ledcAttachPin(ciMotorRightB, 4);

  ledcSetup(1, 20000, 8); // 20mS PWM, 8-bit resolution
  ledcSetup(2, 20000, 8);
  ledcSetup(3, 20000, 8);
  ledcSetup(4, 20000, 8);
}

void driveLeftSide(int power) {
  if (power > 0) {
    ledcWrite(1, min(power, driveMaxPower));
    ledcWrite(2, 0);
  } else if (power < 0) {
    ledcWrite(1, 0);
    ledcWrite(2, min(power, driveMaxPower));
  } else {
    ledcWrite(1, 0);
    ledcWrite(2, 0);
  }
}

void driveRightSide(int power) {
  if (power > 0) {
    ledcWrite(3, min(power, driveMaxPower));
    ledcWrite(4, 0);
  } else if (power < 0) {
    ledcWrite(3, 0);
    ledcWrite(4, min(power, driveMaxPower));
  } else {
    ledcWrite(3, 0);
    ledcWrite(4, 0);
  }
}

void drive(int power) {
  driveLeftSide(power);
  driveRightSide(power);
}

void drive(int leftPower, int rightPower) {
  driveLeftSide(leftPower);
  driveRightSide(rightPower);
}

void toggleDrive() {
  if (curDriveState == STOP) {
    changeState(driveManeuvers[driveManeuverIndex].state);
  } else {
    changeState(STOP);
  }
}

bool moveStraightTo(int cmTarget) {
//  const double& kP = drivekP;
//  const double& kI = drivekI;
//  const double& accelTime = driveAccelTime;
//  const double& steerkP = driveSteerkP;

  const int target = cmToEnc(cmTarget);
  int& distError = error1;
  int& steerError = error2;

  distError = target - (ENC_vi32LeftOdometer + ENC_vi32RightOdometer) / 2;
  steerError = ENC_vi32LeftOdometer - ENC_vi32RightOdometer;

  int& power = power1;
  int& steerPower = power2;
  int& distP = proportional1;
  int& steerP = proportional2;
  double& distIntegral = integral;

  if (millis() < driveStateTime + driveAccelTime) {
    distP = 0;
    power = map(millis() - driveStateTime, 0, driveAccelTime, 0, driveMaxPower);
  } else {
    distP = distError * drivekP;
    power = distP + distIntegral;
  }  
  
  steerP = steerError * driveSteerkP;
  steerPower = steerP;

  int leftPower = max(0, power + steerPower);
  int rightPower = max(0, power - steerPower);

  if (distError >= 5)
    drive(leftPower, rightPower);
  else
    return true;

  distIntegral += drivekI;
  return false;
}

bool turnTo(int _target, bool cw) {
  const double kP = 8.5;
  const double kI = 0.15;
  const double accelTime = 400;

  const int target = _target;

  int error = target - ((abs(ENC_vi32LeftOdometer) + abs(ENC_vi32RightOdometer)) / 2);
  int leftPower = (error * kP + kI) * (cw ? -1 : 1);
  int rightPower = (error * kP + kI) * (cw ? 1 : -1);

  if (error >= 5)
    drive(leftPower, rightPower);
  else
    return true;

  integral += kI;
  return false;
}

void handleDrive() {
  bool inMotionAlg = curDriveState == DRIVE || curDriveState == TURN;
  bool printing =  millis() < driveStateTime + printTime;

  if (inMotionAlg || printing) {
    if (inMotionAlg) {
      Serial.printf("IN ALG.   | ");
    } else if (printing) {
      Serial.printf("DONE ALG. | ");
    }
    Serial.printf("E1: %3d, E2: %3d, P1: %3d, P2: %3d, p1: %3d, p2: %3d, i: %f\n", error1, error2, power1, power2, proportional1, proportional2, integral);
  }

  switch (curDriveState) {
    case STOP:
      drive(0);
      break;
    case DRIVE:
      if (moveStraightTo(25))
        changeState(BRAKE);
      break;
    case TURN:
      if (turnTo(enc90turn, cwNavigation))
        changeState(BRAKE);
      break;
    case BRAKE:
      driveState state = driveManeuvers[driveManeuverIndex].state;
      int target = driveManeuvers[driveManeuverIndex].target;

      if (state == DRIVE) {
        drive(-brakePower * sgn(target));
      } else if (state == TURN) {
        int i = cwNavigation ? 1 : -1;
        drive(-brakePower * i, brakePower * i);
      }

      if (millis() > driveStateTime + brakeTime) {
        if (driveManeuverIndex < nDriveManeuvers - 1) {
          driveManeuverIndex++;
          changeState(driveManeuvers[driveManeuverIndex].state);
        } else {
          changeState(STOP);
        }
      }
  }
}

#endif
