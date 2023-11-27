#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int TXPin = 4, RXPin = 3;
static const uint32_t GPSBaud = 9600;
RF24 radio(7, 8); // CE, CSN

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

const byte address[6] = {0xe1, 0xf0, 0xf0, 0xf0, 0xf0}; //"00001"

double p1Coordinates[2] = { 2, 2 }; //for now, will write to class later.
double targetAngle, xDist, yDist;

double convToDegrees(double radNum) {
  return (radNum * (180/M_PI));
}

double convToRadian(double degNum) {
  return (degNum * (M_PI/180));
}

double angleFromCoordinate(double lat1, double long1, double lat2, double long2) {

    double dLon = (long2 - long1);

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    double brng = atan2(y, x);

    brng = convToDegrees(brng);
    brng = fmod((brng + 360), 360);
    //brng = 360 - brng; // count degrees counter-clockwise - remove to make clockwise

    return brng;
}

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) { //& ss.available()) {
    /* if (gps.encode(ss.read())) {
      p1Coordinates[0] = gps.location.lat();
      p1Coordinates[1] = gps.location.lng();
    } */

    //Serial.println("LAT: " + String(p1Coordinates[0]) + ", LNG: " + String(p1Coordinates[1])); 

    double p2Coordinates[2];
    
    p1Coordinates[0] = 29.5815637;
    p1Coordinates[1] = -98.617998;
    
    radio.read(&p2Coordinates, sizeof(p2Coordinates));
    //Serial.println("x2: " + String(p2Coordinates[0]) + ", y2: " + String(p2Coordinates[1]));

    targetAngle = angleFromCoordinate(convToRadian(p1Coordinates[0]), convToRadian(p1Coordinates[1]), convToRadian(p2Coordinates[0]), convToRadian (p2Coordinates[1]));

    /* yDist = p2Coordinates[1] - p1Coordinates[1];
    xDist = p2Coordinates[0] - p1Coordinates[0];
    targetAngle = atan2(yDist, xDist); //radians
    
    if (targetAngle < 0) //for [0, PI] range
      targetAngle += 2 * M_PI;

    targetAngle *= (180/M_PI); //convert to degrees */

    Serial.println("Angle: " + String(targetAngle));
  }
}
