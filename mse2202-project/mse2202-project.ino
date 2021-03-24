/* Sensor Pins */
const int buttonPin = 27;
const int potiPin = 32;
const int currentSensorPin = A5;

/* Motor Pins */
const int climbMotorFPin = 15;
const int climbMotorRPin = 2;

/* Sensor State/Time Variables */
int button = 1;
int buttonLast = 1;

int poti = 0;

int current = 0;
long currentChangeTime = 0;
const int currentThreshold = 1700;
const int currentStallTime = 250; 

/* Motor State/Time Variables */
bool motorOn = false;
const int minMotorSpeed = 150;
const int maxMotorSpeed = 255;
int motorSpeed = 0;

void setup() {
   // Sensor pin setup
   pinMode(buttonPin, INPUT_PULLUP);
   pinMode(potiPin, INPUT);
   pinMode(currentSensorPin, INPUT); // Current sensor
  
   // ledc (LED Control) pins setup, used as PWMs for motors
   ledcAttachPin(climbMotorFPin, 1); // F means forward
   ledcAttachPin(climbMotorRPin, 2); // R means reverse
   ledcSetup(1, 20000, 8);
   ledcSetup(2, 20000, 8);

   Serial.begin(115200);
}

void loop() {
  button = digitalRead(buttonPin);
  poti = analogRead(potiPin);
  current = analogRead(currentSensorPin);
  motorSpeed = map(poti, 0, 4095, minMotorSpeed, maxMotorSpeed);

  Serial.printf("Button: %d, Poti: %d, Current: %d\n", button, poti, current);
  
  if (button == 1 && buttonLast == 0) // Button has just been pressed
    motorOn = !motorOn;               // Turn the motor on if off, turn motor off if on

  if (current >= currentThreshold) {
    currentChangeTime = 0;
  } else if (currentChangeTime != 0 && millis() > currentChangeTime) {
    motorOn = false;
  } else if (currentChangeTime == 0 && current < currentThreshold) {
    currentChangeTime = millis() + currentStallTime;
  } 

  if (motorOn) {
    ledcWrite(1, motorSpeed);
    ledcWrite(2, 0);
  } else {
    ledcWrite(1, 0);
    ledcWrite(2, 0);
  }

  buttonLast = button;
}
