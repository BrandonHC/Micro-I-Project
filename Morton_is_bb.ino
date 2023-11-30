#include <PID_v1_bc.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define FREQUENCY 50
#define RX_PIN 3  // RX Pin
#define TX_PIN 1  // TX Pin

uint8_t servo = 0; // Servo Instantiation

// Define pins
const int encoderPinA = 32;                // Replace with your encoder pin
const int encoderPinB = 33;                // Replace with your encoder pin
const int switchPin = 25;                  // define the input pin for the encoder switch
const int fullRevolutionValue240 = 240;    // Encoder units for one full revolution
const int fullRevolutionValue360 = 360;    // Degrees in one full revolution

//float receivedData;
byte startByte = 0x7E; // Start byte '~'
byte endByte = 0x7F;   // End byte DEL

// Encoder parameters
int encoderPos240 = 0;                    // Encoder position for 240-unit cycle
int encoderPos360 = 0;                    // Encoder position for 360-degree cycle
int lastEncoded = 0;
int switchState = HIGH;
int lastSwitchState = HIGH;

// PID parameters
double Kp = 9.0;                    // Proportional constant
double Ki = 1.0;                    // Integral Constant
double Kd = 0.0;                    // Derivative Constant
double targetAngle = 0;             // Target angle
double currentAngle = 0;            // Current angle from encoder
double angle;                       // Encoder reading placeholder
double output = 0;                  // PID output

union FloatByteConverter {
  float floatValue;
  byte byteArray[sizeof(float)];
};

SemaphoreHandle_t xMutex;
PID control(&angle, &output, &targetAngle, Kp, Ki, Kd, DIRECT); // PID object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int readEncoder();
void readEncoderTask(void *pvParameters);

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Initialize Serial1 on ESP32

  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  // Initialize PID controller
  control.SetMode(AUTOMATIC); // AUTOMATIC
  control.SetOutputLimits(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); // PWM output range in us

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(switchPin, INPUT_PULLUP); // configure switch pin with internal pull-up resistor

  xMutex = xSemaphoreCreateMutex();

  // Create a task for reading the encoder
  xTaskCreatePinnedToCore(
    readEncoderTask,      // Task function
    "ReadEncoderTask",    // Name of the task
    10000,                // Stack size
    NULL,                 // Task input parameter
    1,                    // Priority of the task
    NULL,                 // Task handle
    0);
}

void loop() {
  // Safely read the shared variable
  xSemaphoreTake(xMutex, portMAX_DELAY);
  angle = currentAngle;
  xSemaphoreGive(xMutex);

  if (Serial1.available() >= sizeof(float) + 2) {
    FloatByteConverter receivedData;

    // Wait for the start byte
    while (Serial1.available() && Serial1.read() != startByte) {}

    // Read the byte array into the float variable
    Serial1.readBytes(receivedData.byteArray, sizeof(float));

    // Wait for the end byte
    while (Serial1.available() && Serial1.read() != endByte) {}

    // Convert the byte array to a string for comparison
    String receivedString = String((char*)receivedData.byteArray);

    if (receivedString != "ovf" && receivedString != "nan" && receivedString != "-0.00") {
      // Convert the byte array to a float and assign it to targetAngle
      float targetAngle = receivedData.floatValue;
      Serial.print("Received angle: ");
      Serial.println(targetAngle);
    } else {
      // Handle the special cases (ovf, nan, -0.00)
      Serial.println("Special case received: " + receivedString);
    }
  }

  // Compute the PID output
  control.Compute();

  // Adjust servo speed based on PID output
  int pcaValue = map(output, 0, 1000000 / FREQUENCY, 0, 4096); 
  pwm.setPWM(servo, 0, pcaValue);

  // Debugging: Print the encoder reading and PID output
  Serial.print("Current Angle: ");
  Serial.print(angle);
  Serial.print(", PID Output: ");
  Serial.println(output);
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
  /*
    if (encoderPos240 >= fullRevolutionValue240) {
    encoderPos240 = 0;
  } else if (encoderPos240 < 0) {
    encoderPos240 = fullRevolutionValue240 - 1;
  }
  */

  // Map encoderPos240 to encoderPos360 for a 360-degree cycle
  encoderPos360 = map(encoderPos240, 0, fullRevolutionValue240, 0, fullRevolutionValue360);
  lastEncoded = encoded; // Save the current encoder state for the next iteration

  return encoderPos360;
}

void readEncoderTask(void *pvParameters)  {
  while (1) {
    int angle = readEncoder();

    // Aquire mutex and update shared variable
    xSemaphoreTake(xMutex, portMAX_DELAY);
    currentAngle = angle;
    xSemaphoreGive(xMutex);

    vTaskDelay(pdMS_TO_TICKS(10));
    }
}