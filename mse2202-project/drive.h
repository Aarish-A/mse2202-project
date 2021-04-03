#ifndef DRIVE_H
#define DRIVE_H 1

const int driveMaxPower = 255;
const int encToRotRatio = 120;
const int rotToCMRatio = 4.2 * 3.14159;

unsigned long driveStopInterval = 250;
unsigned long driveStateTime = 0;

enum driveStates {
  STOPPED = 0,
  FIRST_FORWARD,
  FIRST_TURN,
  SECOND_FORWARD,
  SECOND_TURN,
  THIRD_FORWARD
};

driveStates curDriveState = STOPPED;
driveStates nextDriveState = STOPPED;
driveStates prevDriveState = STOPPED;

void changeState(driveStates nextState) {
  prevDriveState = curDriveState;
  curDriveState = STOPPED;
  nextDriveState = nextState;
  driveStateTime = millis();
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
    ledcWrite(1, max(power, driveMaxPower));
    ledcWrite(2, 0);
  } else if (power < 0) {
    ledcWrite(1, 0);
    ledcWrite(2, max(power, driveMaxPower));
  } else {
    ledcWrite(1, 0);
    ledcWrite(2, 0);
  }
}

void driveRightSide(int power) {
  if (power > 0) {
    ledcWrite(3, max(power, driveMaxPower));
    ledcWrite(4, 0);
  } else if (power < 0) {
    ledcWrite(3, 0);
    ledcWrite(4, max(power, driveMaxPower));
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
  ENC_ClearOdometer();
  changeState((driveStates)((int)STOPPED + 1));
}

void stopDrive() {
  changeState(STOPPED);
}

bool moveStraightTo(int cmTarget) {
  const double kP = 3.1;
  const double steerkP = 2;
  const double accelTime = 400;

  const int target = cmToEnc(cmTarget);

  int distError = target - (ENC_vi32LeftOdometer + ENC_vi32RightOdometer) / 2;
  int steerError = ENC_vi32LeftOdometer - ENC_vi32RightOdometer;
  
  int power;
  if (millis() < driveStateTime + accelTime)
    power = map(millis() - driveStateTime, 0, accelTime, 150, driveMaxPower);
  else
    power = distError * kP;
  int steerCorrectPower = steerError * steerkP;

  Serial.printf("Target: %d, Left: %d, Right: %d, St. Error: %d, St. Power: %d, Error: %d, Power: %d\n | %s", target, ENC_vi32LeftOdometer, ENC_vi32RightOdometer, steerError, steerCorrectPower, steerError, power, millis() < driveStateTime + accelTime ? "ACCEL." : "");

  if (distError >= 5)
    drive(power - steerCorrectPower, power + steerCorrectPower);
  else
    return true;

  return false;
}

void handleDrive() {
  switch (curDriveState) {
    case STOPPED:
      drive(0);
      if (millis() > driveStateTime + driveStopInterval)
        curDriveState = nextDriveState;
      break;
    case FIRST_FORWARD:
      if (moveStraightTo(35)) {
        stopDrive();
      }
      drive(255);
      break;
  }
}

#endif
