#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int encoderPinA = 2;
int encoderPinB = 3;
int switchPin = 4;
int encoderPos240 = 0;  // Encoder position for 240-unit cycle
int encoderPos360 = 0;  // Encoder position for 360-degree cycle
int lastEncoded = 0;
int switchState = HIGH;
int lastSwitchState = HIGH;
int fullRevolutionValue240 = 240;  // Encoder units for one full revolution
int fullRevolutionValue360 = 360; // Degrees in one full revolution

void setup() {
  pwm.begin();
  pwm.setPWMFreq(60);
  
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(switchPin, INPUT_PULLUP);
  
  Serial.begin(9600);
}

void loop() {
  switchState = digitalRead(switchPin);

  if (switchState != lastSwitchState) {
    if (switchState == LOW) {
      // Switch is pressed
      // Add your code for handling the switch press here
    }
  }

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

  // Map encoderPos360 to PWM values for the servo
  int targetPos = map(encoderPos360, 0, fullRevolutionValue360, 150, 600);
  pwm.setPWM(0, 0, targetPos);

  Serial.print("Encoder Position (240): ");
  Serial.print(encoderPos240);
  Serial.print("\t Encoder Position (360): ");
  Serial.println(encoderPos360);

  // Your additional code can go here
}
