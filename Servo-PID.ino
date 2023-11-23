#include <Servo.h>
#include <PID_v1_bc.h>

// Define pins
const int encoderPinA = 2;  // Replace with your encoder pin
const int encoderPinB = 3;  // Replace with your encoder pin
const int servoPin = 9;     // Replace with your servo pin
const int switchPin = 4;  // define the input pin for the encoder switch

// Encoder parameters
int encoderPos = 0; // variable to store the encoder position
int lastEncoded = 0;
int switchState = HIGH;  // variable to store the switch state
int lastSwitchState = HIGH;  // variable to store the previous switch state

// PID parameters
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;
double targetAngle = 90;  // Target angle
double currentAngle = 0;  // Current angle from encoder
double output = 0;        // PID output

Servo myservo;  // Create servo object
PID control(&currentAngle, &output, &targetAngle, Kp, Ki, Kd, DIRECT); // PID object


void setup() {
  myservo.attach(9); // attach the servo on pin 9 to the servo object
  myservo.write(90); // set the initial servo position to 90 degrees

  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Assuming -255 to 255 is your speed range


  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(switchPin, INPUT_PULLUP); // configure switch pin with internal pull-up resistor
  Serial.begin(9600);
}

void loop() {
  // Read encoder to find current angle
  double currentAngle = map(readEncoder(), 0, 240, 0, 360);

  // Compute the PID output
  myPID.Compute();

  // Adjust the servo speed based on the PID output
  int servoSpeed = map(output, -255, 255, 0, 180); // Map to servo's speed range
  myServo.write(servoSpeed);
}

int readEncoder() {
  // Read the switch state
  switchState = digitalRead(switchPin);

  // Check for a change in the switch state
  if (switchState != lastSwitchState) {
    if (switchState == LOW) {
      // The switch is pressed
    }
  }

  lastSwitchState = switchState; // Save the current switch state for the next iteration

  // Read the encoder state
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // Update encoder position based on the state change
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos--;
  }

  lastEncoded = encoded; // Save the current encoder state for the next iteration

  return encoderPos;
}
