#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(0, 1); // RX, TX

void setup() {
  Serial.begin(9600);
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
      float latitudeDegrees = latitudeStr.substring(0, 2).toFloat();
      float latitudeMinutes = latitudeStr.substring(2).toFloat();
      float latitude = latitudeDegrees + (latitudeMinutes / 60.0);

      // Convert longitude to Degrees and Decimal Degrees
      float longitudeDegrees = longitudeStr.substring(0, 3).toFloat();
      float longitudeMinutes = longitudeStr.substring(3).toFloat();
      float longitude = longitudeDegrees + (longitudeMinutes / 60.0);

      // Determine the hemisphere for longitude (W or E)
      String longitudeHemisphere = values[5];

      if (longitudeHemisphere == "W") {
        longitude = -longitude; // Make longitude negative for western hemisphere
      }

      Serial.print("Latitude: ");
      Serial.println(latitude, 6);  // Print with 6 decimal places
      Serial.print("Longitude: ");
      Serial.println(longitude, 6); // Print with 6 decimal places
    }
  }
}
