#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = {0xe1, 0xf0, 0xf0, 0xf0, 0xf0}; //"00001"

float p1Coordinates[2] = { 0, 0 }; //for now, will write to class later.
float targetAngle, xDist, yDist;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    float p2Coordinates[2];
    radio.read(&p2Coordinates, sizeof(p2Coordinates));
    Serial.println("x2: " + String(p2Coordinates[0]) + ", y2: " + String(p2Coordinates[1]));

    yDist = p2Coordinates[1] - p1Coordinates[1];
    xDist = p2Coordinates[0] - p1Coordinates[0];
    targetAngle = atan2(yDist, xDist); //radians
    
    if (targetAngle < 0) //for [0, PI] range
      targetAngle += 2 * M_PI;

    targetAngle *= (180/PI);

    Serial.println("Angle: " + String(targetAngle));
  }
}
