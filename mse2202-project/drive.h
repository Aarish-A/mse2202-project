#ifndef DRIVE_H
#define DRIVE_H 1


const int encToRotRatio = 120;
const int rotToCMRatio = 4.2 * 3.14159;
const int enc90turn = 21;
const bool cwNavigation = true;

const int driveMaxPower = 255;

unsigned long driveStopInterval = 750;
unsigned long driveStateTime = 0;

double moveIntegral = 0;

enum driveStates {
  STOPPED = 0,
  FIRST_FORWARD,
  FIRST_TURN,
  SECOND_FORWARD,
  SECOND_TURN,
  THIRD_FORWARD,
  FORWARD_BRAKE
};

driveStates curDriveState = STOPPED;
driveStates nextDriveState = STOPPED;
driveStates prevDriveState = STOPPED;

void changeState(driveStates nextState) {
  prevDriveState = curDriveState;
  curDriveState = STOPPED;
  nextDriveState = nextState;
  driveStateTime = millis();
  ENC_ClearOdometer();
  moveIntegral = 0;
}

void changeStateForwardBrake(driveStates nextState) {
  prevDriveState = curDriveState;
  curDriveState = FORWARD_BRAKE;
  nextDriveState = nextState;
  driveStateTime = millis();
  ENC_ClearOdometer();
  moveIntegral = 0;
}

int cmToEnc(int cm) {
  return cm / rotToCMRatio * encToRotRatio;
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

void startDrive() {
  changeState((driveStates)((int)STOPPED + 1));
}

void stopDrive() {
  changeState(STOPPED);
}

bool moveStraightTo(int cmTarget) {
  const double kP = 2.4;
  const double kI = 2.1;
  const double accelTime = 800;
  double steerkP = 8.9;

  const int target = cmToEnc(cmTarget);

  int distError = target - (ENC_vi32LeftOdometer + ENC_vi32RightOdometer) / 2;
  int steerError = ENC_vi32LeftOdometer - ENC_vi32RightOdometer;

  int power;
  if (millis() < driveStateTime + accelTime)
    power = map(millis() - driveStateTime, 0, accelTime, 0, driveMaxPower);
  else
    power = distError * kP + moveIntegral;

  if (distError <= 15)
    steerkP = 13.5;
  int steerCorrectPower = steerError * steerkP;

  Serial.printf("Target: %d, Left: %d, Right: %d, St. Error: %d, St. Power: %d, Error: %d, I: %d, Power: %d\n", target, ENC_vi32LeftOdometer, ENC_vi32RightOdometer, steerError, steerCorrectPower, distError, (int)moveIntegral, power);

  int leftPower = max(0, power + steerCorrectPower);
  int rightPower = max(0, power - steerCorrectPower);

  if (distError >= 5)
    drive(leftPower, rightPower);
  else
    return true;

  moveIntegral += kI;
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

  Serial.printf("Target: %d, L: %d, R: %d, Error: %d, Integral: %d, Power: %d\n", target, ENC_vi32LeftOdometer, ENC_vi32RightOdometer, error, (int)moveIntegral, abs(leftPower));

  if (error >= 5)
    drive(leftPower, rightPower);
  else
    return true;

  moveIntegral += kI;
  return false;
}

void handleDrive() {
  switch (curDriveState) {
    case STOPPED:
      drive(0);
      if (millis() > driveStateTime + driveStopInterval) {
        curDriveState = nextDriveState;
        ENC_ClearOdometer();
      }
      break;
    case FIRST_FORWARD:
      if (moveStraightTo(25))
        changeStateForwardBrake(FIRST_TURN);
      break;
    case FIRST_TURN:
      if (turnTo(enc90turn, cwNavigation))
        changeState(SECOND_FORWARD);
      break;
    case SECOND_FORWARD:
      if (moveStraightTo(30))
        changeStateForwardBrake(SECOND_TURN);
      break;
    case SECOND_TURN:
      if (turnTo(enc90turn, cwNavigation))
        changeState(THIRD_FORWARD);
      break;
    case THIRD_FORWARD:
      if (moveStraightTo(25))
        stopDrive();
      break;
    case FORWARD_BRAKE:
      drive(-40);
      if (millis() > driveStateTime + 75) {
        changeState(nextDriveState);
      }
  }
}

#endif
