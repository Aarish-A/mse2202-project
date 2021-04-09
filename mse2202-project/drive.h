#ifndef DRIVE_H
#define DRIVE_H 1

#include "tuning.h"
#include "util.h"

// Global Movement Measuring Variables
const int printTime = 1500;

int target = 0;
int error1 = 0;
int error2 = 0;
int power1 = 0;
int power2 = 0;

int proportional1 = 0;
int proportional2 = 0;
double integral = 0;

// Structures and Enums
const int nDriveManeuvers = sizeof(driveManeuvers) / sizeof(driveManeuver);

// Drive State and Maneuver Management Variables
unsigned long driveStateTime = 0;
driveState curDriveState = STOP;
driveState prevDriveState = STOP;
unsigned int driveManeuverIndex = 0;

void resetMeasurements() {
  ENC_ClearOdometer();
  target = 0;
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
    case STOP:
      Serial.printf("Switched state to STOP, took %lu time\n", millis() - driveStateTime);
      driveManeuverIndex = 0;
      break;
    case DRIVE:
      Serial.printf("Switched state to DRIVE, took %lu time\n", millis() - driveStateTime);
      resetMeasurements();
      break;
    case TURN:
      Serial.printf("Switched state to TURN, took %lu time\n", millis() - driveStateTime);
      resetMeasurements();
      break;
    case BRAKE:
      Serial.printf("Switched state to BRAKE, took %lu time\n", millis() - driveStateTime);
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

bool readyToClimb() {
  if (driveManeuverIndex == nDriveManeuvers - 1)
    return true;
}

bool driveTo(int cmTarget) {
  target = cmToEnc(cmTarget);
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

  if (distError >= 1)
    drive(leftPower, rightPower);
  else
    return true;

  distIntegral += drivekI;
  return false;
}

bool turnTo(int degTarget, bool cw) {
  target = degTurnToEnc(degTarget);

  int& distEerror = error1;
  int& wheelError = error2;

  distEerror = target - ((abs(ENC_vi32LeftOdometer) + abs(ENC_vi32RightOdometer)) / 2);
  wheelError = abs(ENC_vi32LeftOdometer) - abs(ENC_vi32RightOdometer);

  int& leftPower = power1;
  int& rightPower = power2;
  int& p = proportional1;
  proportional2 = 0;
  double& turnIntegral = integral;
  int i = cwNavigation ? -1 : 1;

  if (millis() < driveStateTime + driveAccelTime) {
    p = 0;
    leftPower = map(millis() - driveStateTime, 0, driveAccelTime, 0, driveMaxPower) * i;
    rightPower = map(millis() - driveStateTime, 0, driveAccelTime, 0, driveMaxPower) * -i;
  } else {
    p = distEerror * turnkP;
    leftPower = (p + turnIntegral) * i;
    rightPower = (p + turnIntegral) * -i;
  }
  
  if (distEerror >= 2)
    drive(leftPower, rightPower);
  else
    return true;

  turnIntegral += turnkI;
  return false;
}

void handleDrive() {
  bool inMotionAlg = curDriveState == DRIVE || curDriveState == TURN;
  bool printing =  millis() < driveStateTime + printTime;

  if (inMotionAlg || printing) {
    if (inMotionAlg) {
      Serial.printf("IN ALG.   | ");
    } else if (printing) {
      error1 = target - (abs(ENC_vi32LeftOdometer) + abs(ENC_vi32RightOdometer)) / 2;
      error2 = abs(ENC_vi32LeftOdometer) - abs(ENC_vi32RightOdometer);
      Serial.printf("DONE ALG. | ");
    }
    Serial.printf("T: %d, E1: %3d, E2: %3d, P1: %3d, P2: %3d, p1: %3d, p2: %3d, i: %f\n", target, error1, error2, power1, power2, proportional1, proportional2, integral);
  }

  switch (curDriveState) {
    case STOP:
      power1 = 0;
      power2 = 0;
      drive(power1);
      break;
    case DRIVE:
      if (driveTo(driveManeuvers[driveManeuverIndex].target))
        changeState(BRAKE);
      break;
    case TURN:
      if (turnTo(driveManeuvers[driveManeuverIndex].target, cwNavigation))
        changeState(BRAKE);
      break;
    case BRAKE:
      driveState state = driveManeuvers[driveManeuverIndex].state;
      int target = driveManeuvers[driveManeuverIndex].target;

      if (state == DRIVE) {
        int power = -brakePower * sgn(target);
        power1 = power * pow((double)ENC_vi32LeftOdometer / ENC_vi32RightOdometer, 2.2);
        power2 = power * pow((double)ENC_vi32RightOdometer / ENC_vi32LeftOdometer, 2.2);
        drive(power1, power2);
      } else if (state == TURN) {
        int i = cwNavigation ? -1 : 1;
        power1 = -brakePower * i;
        power2 = brakePower * i;
        drive(power1, power2);
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
