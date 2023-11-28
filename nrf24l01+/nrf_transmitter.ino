#include <SoftwareSerial.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
SoftwareSerial gpsSerial(0, 1); // RX, TX

const byte address[6] = {0xe1, 0xf0, 0xf0, 0xf0, 0xf0}; //"00001"
double origin[2] = { 0, 0 };

void setup() { //nanofarads
  Serial.begin(9600);
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening();

  gpsSerial.begin(9600);
}

void loop() {
  if (gpsSerial.available() > 0) {
    // Read a line from the GPS module
    String line = gpsSerial.readStringUntil('\n');

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

      // Extract latitude and longitude
      String latitudeStr = values[2];
      String longitudeStr = values[4];

      // Convert latitude to Degrees and Decimal Degrees
      double latitudeDegrees = latitudeStr.substring(0, 2).toDouble();
      double latitudeMinutes = latitudeStr.substring(2).toDouble();
      double latitude = latitudeDegrees + (latitudeMinutes / 60.0);

      // Convert longitude to Degrees and Decimal Degrees
      double longitudeDegrees = longitudeStr.substring(0, 3).toDouble();
      double longitudeMinutes = longitudeStr.substring(3).toDouble();
      double longitude = longitudeDegrees + (longitudeMinutes / 60.0);

      // Determine the hemisphere for longitude (W or E)
      String longitudeHemisphere = values[5];

      if (longitudeHemisphere == "W")
        longitude = -longitude; // Make longitude negative for western hemisphere

      origin[0] = latitude;
      origin[1] = longitude;
      }
    }
  radio.write(&origin, sizeof(origin));
  delay(1000);
}
