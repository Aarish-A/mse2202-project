/* Sensor Pins */
const int buttonPin = 27;
const int potiPin = 32;

/* Motor Pins */
const int climbMotorFPin = 15;
const int climbMotorRPin = 2;

/* Sensor State/Time Variables */
int button = 1;
int buttonLast = 1;
int poti = 0;

/* Motor State/Time Variables */
bool motorOn = false;
const int minMotorSpeed = 150;
const int maxMotorSpeed = 255;
int motorSpeed = 0;

void setup() {
   // Sensor pin setup
   pinMode(buttonPin, INPUT_PULLUP);
   pinMode(potiPin, INPUT);
  
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
  motorSpeed = map(poti, 0, 4095, minMotorSpeed, maxMotorSpeed);
  
  Serial.print("Button: ");
  Serial.print(button);
  Serial.print(" Poti: ");
  Serial.print(poti);
  Serial.print(" Motor: ");
  Serial.println(motorOn);
  
  if (button == 1 && buttonLast == 0) // Button has just been pressed
    motorOn = !motorOn;               // Turn the motor on if off, turn motor off if on

  if (motorOn) {
    ledcWrite(1, motorSpeed);
    ledcWrite(2, 0);
  } else {
    ledcWrite(1, 0);
    ledcWrite(2, 0);
  }

  buttonLast = button;
}
