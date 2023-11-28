#include <PID_v1_bc.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define FREQUENCY 50

uint8_t servo = 0; // Servo Instantiation

// Define pins
const int encoderPinA = 32;                // Replace with your encoder pin
const int encoderPinB = 33;                // Replace with your encoder pin
const int switchPin = 25;                  // define the input pin for the encoder switch
const int fullRevolutionValue240 = 240;   // Encoder units for one full revolution
const int fullRevolutionValue360 = 360;   // Degrees in one full revolution

// Encoder parameters
int encoderPos240 = 0;                    // Encoder position for 240-unit cycle
int encoderPos360 = 0;                    // Encoder position for 360-degree cycle
int lastEncoded = 0;
int switchState = HIGH;
int lastSwitchState = HIGH;

// PID parameters
double Kp = 9.0;                    // Proportional constant
double Ki = 0.0;                    // Integral Constant
double Kd = 0.0;                    // Derivative Constant
double targetAngle = 90;            // Target angle
double currentAngle = 0;            // Current angle from encoder
double output = 0;                  // PID output

PID control(&currentAngle, &output, &targetAngle, Kp, Ki, Kd, DIRECT); // PID object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  // Initialize PID controller
  control.SetMode(AUTOMATIC); //AUTOMATIC
  control.SetOutputLimits(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); // PWM output range in us


  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(switchPin, INPUT_PULLUP); // configure switch pin with internal pull-up resistor
  Serial.begin(9600);
}

void loop() {
  // Read encoder to find current angle
  control.myInput = (double*)readEncoder();

  // Compute the PID output
  control.Compute();

  // Adjust servo speed based on PID output
  int pcaValue = map(output, 0, 1000000 / FREQUENCY, 0, 4096); 
  pwm.setPWM(servo, 0, pcaValue);

  // Debugging: Print the encoder reading and PID output
  Serial.print("Current Angle: ");
  Serial.print(*control.myInput);  
  Serial.print(", PID Output: ");
  Serial.println(output);

  delay(100);
}

int readEncoder() {
  switchState = digitalRead(switchPin);
  lastSwitchState = switchState;

  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos240++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos240--;
  }

  lastEncoded = encoded;

  // Ensure encoderPos240 stays within a full revolution
  if (encoderPos240 >= fullRevolutionValue240) {
    encoderPos240 = 0;
  } else if (encoderPos240 < 0) {
    encoderPos240 = fullRevolutionValue240 - 1;
  }

  // Map encoderPos240 to encoderPos360 for a 360-degree cycle
  encoderPos360 = map(encoderPos240, 0, fullRevolutionValue240, 0, fullRevolutionValue360);
  lastEncoded = encoded; // Save the current encoder state for the next iteration

  return encoderPos360;
}