#ifndef DRIVE_H
#define DRIVE_H 1

#include "util.h"
#include "tuning.h"

// Global movement measuring variables
// These are context dependent and not local to easily graph in the web server
const int printTime = 1500; // How long to print the measurement variables after finishing a movement

int target = 0;
int error1 = 0;
int error2 = 0;
int power1 = 0;
int power2 = 0;

int proportional1 = 0;      // Proportional portion of PI loop
int proportional2 = 0;
double integral = 0;        // Integral portion of PI loop

// Maneuver management variables
const int nDriveManeuvers = sizeof(driveManeuvers) / sizeof(driveManeuver);     // Number of drive maneuvers declared in "tuning.h"
unsigned int driveManeuverIndex = 0;

// Drive state management variables
unsigned long driveStateTime = 0;
driveState curDriveState = STOP;

// Reset global measuring variables
void resetMeasurements(void) {
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

// Setup motors and LEDC channels for drive
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

// Power the left drive motor between -255 to 255
void driveLeftSide(int power) {
  if (power > 0) {
    ledcWrite(1, min(power, abs(driveMaxPower)));
    ledcWrite(2, 0);
  } else if (power < 0) {
    ledcWrite(1, 0);
    ledcWrite(2, min(power, abs(driveMaxPower)));
  } else {
    ledcWrite(1, 0);
    ledcWrite(2, 0);
  }
}

// Power the right drive motor between -255 to 255
void driveRightSide(int power) {
  if (power > 0) {
    ledcWrite(3, min(power, abs(driveMaxPower)));
    ledcWrite(4, 0);
  } else if (power < 0) {
    ledcWrite(3, 0);
    ledcWrite(4, min(power, abs(driveMaxPower)));
  } else {
    ledcWrite(3, 0);
    ledcWrite(4, 0);
  }
}

// Set the drive power for both sides of the robot
void drive(int power) {
  driveLeftSide(power);
  driveRightSide(power);
}

// Function overload for setting the left and right drive powers individually
void drive(int leftPower, int rightPower) {
  driveLeftSide(leftPower);
  driveRightSide(rightPower);
}

/*
 * Algorithm to drive the robot straight to a given centimeters target relative to its current position
 * The power is controlled by a proportional integral (PI) loop, with tuning parameters found in "tuning.h"
 * Local error and power variables are created to make them more contextual, but are set up as a reference to the global measurement variables
 * error1 is determined by difference between the target (centimeters to encoder ticks) and the average of the left/right encoder
 * error2 is determined by the difference between left and right encoders, this value is used to adjust the left/right motor speeds proportionally
 */
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

  if (distError >= 5)
    drive(200, 235);
  else
    return true;

  distIntegral += drivekI;
  return false;
}

/*
 * Algorithm to tank turn the robot to a given angle relative to its current position
 * The power is controlled by a proportional integral (PI) loop, with tuning parameters found in "tuning.h"
 * Local error and power variables are created to make them more contextual, but are set up as a reference to the global measurement variables
 * error1 is determined by difference between the target (angle converted to encoder ticks) and the average of the absolute values of the left/right encoder
 * error2 is determined by the difference between left and right encoders, this value isn't used to power the motors
 */
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
    drive(255, 0);
  else
    return true;

  turnIntegral += turnkI;
  return false;
}

// Change and log the drive state to the given driveState
void changeState(driveState nextState) {
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

// Start the drive if it's stopped, stop the drive if it's running
void toggleDrive() {
  if (curDriveState == STOP) {
    changeState(driveManeuvers[driveManeuverIndex].state);
  } else {
    changeState(STOP);
  }
}

// Returns whether the robot is on its last drive maneuver and ready to climb
bool readyToClimb(void) {
  if (driveManeuverIndex == nDriveManeuvers - 1)
    return true;
}

// Handle the drive state machine based on the current drive state
void handleDrive(void) {
  bool inMotionAlg = curDriveState == DRIVE || curDriveState == TURN;   // Whether the robot is currently driving or turning
  bool printing =  millis() < driveStateTime + printTime;               // Whether to print the measurement variables after finishing a movement

  // Print measurement variables whether in a movement or for a short (printTime) period after exiting a movement
  if (inMotionAlg || printing) {
    if (inMotionAlg) {
      Serial.printf("IN ALG.   | ");
    } else if (printing) {
      error1 = target - (abs(ENC_vi32LeftOdometer) + abs(ENC_vi32RightOdometer)) / 2;     // Distance to target minus average of left/right encoders
      error2 = abs(ENC_vi32LeftOdometer) - abs(ENC_vi32RightOdometer);                    // Difference between left/right encoders
      Serial.printf("DONE ALG. | ");
    }
    Serial.printf("T: %d, E1: %3d, E2: %3d, P1: %3d, P2: %3d, p1: %3d, p2: %3d, i: %f\n", target, error1, error2, power1, power2, proportional1, proportional2, integral);
  }


  switch (curDriveState) {
    case STOP:                                                                              // STOP: set the drive powers to 0
      power1 = 0;
      power2 = 0;
      drive(power1);
      break;
    case DRIVE:                                                                             // DRIVE: drive straight to the next maneuver's target centimeters
      if (driveTo(driveManeuvers[driveManeuverIndex].target))
        changeState(BRAKE);
      break;
    case TURN:                                                                              // TURN: turn to the next maneuver's target angle
      if (turnTo(driveManeuvers[driveManeuverIndex].target, cwNavigation))
        changeState(BRAKE);
      break;
    case BRAKE:                                                                             // BRAKE: brake the motors based on the last maneuver (drive or turn)
      driveState state = driveManeuvers[driveManeuverIndex].state;
      int target = driveManeuvers[driveManeuverIndex].target;

      if (state == DRIVE) {
        // If one encoder is further ahead than another, brake the one further ahead by more
        int power = -brakePower * sgn(target);
        power1 = power * pow((double)ENC_vi32LeftOdometer / ENC_vi32RightOdometer, 2.2);
        power2 = power * pow((double)ENC_vi32RightOdometer / ENC_vi32LeftOdometer, 2.2);
        drive(power1, power2);
      } else if (state == TURN) {
        // Brake the left and right sides opposite of the direction they were moving in the turn
        int i = cwNavigation ? -1 : 1;
        power1 = -brakePower * i;
        power2 = brakePower * i;
        drive(power1, power2);
      }

      // If done braking, go to the next maneuver if there is one or stop the drive
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
