/* Sensor Pins */
const int buttonPin = 27;

/* Motor Pins */
const int climbMotorFPin = 15;
const int climbMotorRPin = 2;

/* Sensor State/Time Variables */
int button = 1;
int buttonLast = 1;

/* Motor State/Time Variables */
bool motorOn = false;

void setup() {
   // Sensor pin setup
   pinMode(buttonPin, INPUT_PULLUP);
  
   // ledc (LED Control) pins setup, used as PWMs for motors
   ledcAttachPin(climbMotorFPin, 1); // F means forward
   ledcAttachPin(climbMotorRPin, 2); // R means reverse
   ledcSetup(1, 20000, 8);
   ledcSetup(2, 20000, 8);

   Serial.begin(115200);
}

void loop() {
  Serial.print("Button: ");
  Serial.print(button);
  Serial.print(" Motor: ");
  Serial.println(motorOn);
  
  button = digitalRead(buttonPin);
  if (button == 1 && buttonLast == 0) // Button has just been pressed
    motorOn = !motorOn;               // Turn the motor on if off, turn motor off if on

  if (motorOn) {
    ledcWrite(1, 255);
    ledcWrite(2, 0);
  } else {
    ledcWrite(1, 0);
    ledcWrite(2, 0);
  }

  buttonLast = button;
}
