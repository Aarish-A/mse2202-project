#ifndef CLIMB_H
#define CLIMB_H 1

const int holdPower = 40;     // Climb motor power when holding at the top
const int upPower = 255;      // Climb motor power when ascending
const int downPower = -255;   // Climb motor power when descending
const long holdTime = 10000;  // Time to hold before descending

const long currentThreshold = 1750;   // Current sensor threshold that determines whether it's been stalled
const long currentStallTime = 250;    // How long the current sensor needs to be stalled for it to be "tripped"
long currentChangeTime = 0;           // How long the current sensor has been stalled for
int current = 0;                      // Current sensor reading

// Climb states for state machine
enum climbState {
  STOPPED = 0,      // Motor off
  UP,               // Motor on for ascension
  DOWN,             // Motor on for descension               
  HOLD              // Motor on for holding at the top
};

// Climb state management variables
unsigned long climbStateTime = 0;
climbState curClimbState = STOPPED;

// Setup sensors, motors, and LEDC channels for climbing
void setupClimb(void) {
  pinMode(ciCurrentSensor, INPUT); // Current sensor

  ledcAttachPin(ciMotorClimbA, 5); // assign Motors pins to channels
  ledcAttachPin(ciMotorClimbB, 6);

  ledcSetup(5, 20000, 8); // 20mS PWM, 8-bit resolution
  ledcSetup(6, 20000, 8);
}

// Power the climb motor between -255 to 255
void climb(int power) {
  if (power > 0) {
    ledcWrite(5, min(255, abs(power)));
    ledcWrite(6, 0);
  } else if (power < 0) {
    ledcWrite(5, 0);
    ledcWrite(6, min(255, abs(power)));
  } else {
    ledcWrite(5, 0);
    ledcWrite(6, 0);
  }
}

// Change and log the climb state to the given climbState
void changeClimbState(climbState nextState) {
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

// Stop the climb by changing the climb state to STOPPED
void stopClimb(void) {
  changeClimbState(STOPPED);
}

// Start the climb by changing the climb state to UP
void startClimb(void) {
  changeClimbState(UP);
}

// Handle the climb state machine based on the current climb state
void handleClimb(void) {
  current = analogRead(ciCurrentSensor);

  if (current <= currentThreshold) {                                      // If the current is underneath the current stall threshold
    currentChangeTime = 0;                                                // set the time the current has stalled for to 0
  } else if (currentChangeTime != 0 && millis() > currentChangeTime) {    // Else if the current has been stalling for more than currentChangeTime (tripped)
    changeClimbState(HOLD);                                               // change the climb state to HOLD 
  } else if (currentChangeTime == 0 && current > currentThreshold) {      // Else if the current sensor just started stalling
    currentChangeTime = millis() + currentStallTime;                      // Set the current stall timeout to currentStallTime milliseconds from now
  } 

  switch (curClimbState) {
    case STOPPED:                                   // STOPPED: set the climb motor to 0 power
      climb(0);
      break;
    case UP:                                        // UP: set the climb motor to the ascent power
      climb(upPower);
      break;
    case DOWN:                                      // DOWN: set the climb motor to the descent power
      climb(-downPower);
      break;
    case HOLD:                                      // HOLD: set the climb motor to the hold power, start descending after holdTime amount of time after entering the state
      climb(holdPower);
      if (millis() > climbStateTime + holdTime)
        changeClimbState(DOWN);
      break;
  }
}

#endif
