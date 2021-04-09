#ifndef CLIMB_H
#define CLIMB_H 1

const int holdPower = 0;
const int upPower = 255;
const int downPower = 100;
const long holdTime = 10000;

int current = 0;
long currentChangeTime = 0;
const long currentThreshold = 1750;
const long currentStallTime = 250; 

enum climbState {
  STOPPED = 0,
  UP,
  DOWN,
  HOLD
};

unsigned long climbStateTime = 0;
climbState prevClimbState = STOPPED;
climbState curClimbState = STOPPED;

void changeClimbState(climbState nextState) {
  prevClimbState = curClimbState;
  curClimbState = nextState;

  switch (curClimbState) {
    case STOPPED:
      Serial.printf("Switched state to STOPPED, took %lu time\n", millis() - climbStateTime);
      break;
    case UP:
      Serial.printf("Switched state to UP, took %lu time\n", millis() - climbStateTime);
      break;
    case DOWN:
      Serial.printf("Switched state to DOWN, took %lu time\n", millis() - climbStateTime);
      break;
    case HOLD:
      Serial.printf("Switched state to HOLD, took %lu time\n", millis() - climbStateTime);
      break;
  }

  climbStateTime = millis();
}

void setupClimb() {
  pinMode(ciCurrentSensor, INPUT); // Current sensor

  ledcAttachPin(ciMotorClimbA, 5); // assign Motors pins to channels
  ledcAttachPin(ciMotorClimbB, 6);

  ledcSetup(5, 20000, 8); // 20mS PWM, 8-bit resolution
  ledcSetup(6, 20000, 8);
}

void climb(int power) {
  if (power > 0) {
    ledcWrite(5, power);
    ledcWrite(6, 0);
  } else if (power < 0) {
    ledcWrite(5, 0);
    ledcWrite(6, power);
  } else {
    ledcWrite(5, 0);
    ledcWrite(6, 0);
  }
}

void stopClimb() {
  changeClimbState(STOPPED);
}

void startClimb() {
  changeClimbState(UP);
}

void handleClimb() {
  current = analogRead(ciCurrentSensor);
//  Serial.printf("Current: %d\n", current);
  
  if (current >= currentThreshold) {
    currentChangeTime = 0;
  } else if (currentChangeTime != 0 && millis() > currentChangeTime) {
    changeClimbState(HOLD);
  } else if (currentChangeTime == 0 && current < currentThreshold) {
    currentChangeTime = millis() + currentStallTime;
  } 
  
  switch (curClimbState) {
    case STOPPED:
      climb(0);
      break;
    case UP:
      climb(upPower);
      break;
    case DOWN:
      climb(-downPower);
      break;
    case HOLD:
      climb(holdPower);
      if (millis() > climbStateTime + holdTime)
        changeClimbState(DOWN);
      break;
  }
}

#endif
