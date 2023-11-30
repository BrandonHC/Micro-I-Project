#include <SoftwareSerial.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
SoftwareSerial gpsSerial(0, 1); // RX, TX
SoftwareSerial mySerial(2, 3); //RX,TX

const byte address[6] = {0xe1, 0xf0, 0xf0, 0xf0, 0xf0}; //"00001"
double p1Coordinates[2] = { 0, 0 }; //for now, will write to class later.
double p1Coordiates[2] = { 0, 0 };
double xDist, yDist;
float targetAngle;
//byte* byteArray;
byte startByte = 0x7E; // Start byte
byte endByte = 0x7F;   // End byte

union FloatByteConverter {
  float floatValue;
  byte byteArray[sizeof(float)];
} packetO;

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

    double brng = atan2(y, x); //bearing angle (radians)

    brng = convToDegrees(brng);
    brng = fmod((brng + 360), 360);
    brng = 360 - brng; // count degrees counter-clockwise - remove to make clockwise

    return brng;
}

void setup() {
  Serial.begin(9600);
  //mySerial.begin(9600);  
  gpsSerial.begin(9600); 
  
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH); //low?
  radio.startListening();
}

void loop() {
  Serial.println("Radio Available State: " + String(radio.available()));
  Serial.println("GPS Available State: " + String(gpsSerial.available()));
  Serial.println("TX Connection State: " + String(mySerial.available()));

  //packetO.floatValue = 144.23;
  //mySerial.write(startByte);
  //mySerial.write(packetO.byteArray, sizeof(float));
  //mySerial.write(endByte);

  //radio.startListening();
  if(radio.available()) {
    double p2Coordinates[2];

    gpsSerial.listen();
    if(gpsSerial.available() > 0) {
      // Read a line from the GPS module
      String line = gpsSerial.readStringUntil('\n');
      //Serial.println(line);
  
      // Check if the line is a valid NMEA sentence
      if (line.startsWith("$GPGGA")) {
        // Parse the NMEA sentence
        String values[15];
        int i = 0;
        int startIndex = 0;
        int endIndex = line.indexOf(',');
  
        while (endIndex != -1) {
          values[i] = line.substring(startIndex, endIndex);
          startIndex = endIndex + 1;
          endIndex = line.indexOf(',', startIndex);
          i++;
          }
          
        String latitudeStr = values[2]; // Extract latitude and longitude
        String longitudeStr = values[4];
  
        double latitudeDegrees = latitudeStr.substring(0, 2).toDouble(); // Convert latitude to Degrees and Decimal Degrees
        double latitudeMinutes = latitudeStr.substring(2).toDouble();
        double latitude = latitudeDegrees + (latitudeMinutes / 60.0);
  
        double longitudeDegrees = longitudeStr.substring(0, 3).toDouble(); // Convert longitude to Degrees and Decimal Degrees
        double longitudeMinutes = longitudeStr.substring(3).toDouble();
        double longitude = longitudeDegrees + (longitudeMinutes / 60.0);
  
        String longitudeHemisphere = values[5]; // Determine the hemisphere for longitude (W or E)
  
        if (longitudeHemisphere == "W")
          longitude = -longitude; // Make longitude negative for western hemisphere
      
        p1Coordinates[0] = latitude;
        p1Coordinates[1] = longitude;

        //radio.startListening();
        radio.read(&p2Coordinates, sizeof(p2Coordinates));
        targetAngle = angleFromCoordinate(convToRadian(p1Coordinates[0]), convToRadian(p1Coordinates[1]), convToRadian(p2Coordinates[0]), convToRadian(p2Coordinates[1]));
        packetO.floatValue = targetAngle;

        Serial.println("Angle: " + String(targetAngle));

        //mySerial.stopListening();
        //mySerial.write(startByte);
        //mySerial.write(packetO.byteArray, sizeof(float));
        //mySerial.write(endByte);
      } 
    }
  }
}
