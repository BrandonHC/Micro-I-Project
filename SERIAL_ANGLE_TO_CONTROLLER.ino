#include <SoftwareSerial.h>

SoftwareSerial mySerial(0, 1); // Define SoftwareSerial object with Tx and Rx pins
//float receivedData;
byte startByte = 0x7E; // Start byte '~'
byte endByte = 0x7F;   // End byte DEL

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600); // Set the baud rate to 9600 (same as sender Arduino)
}

union FloatByteConverter {
  float floatValue;
  byte byteArray[sizeof(float)];
};

void loop() {
  if (mySerial.available() >= sizeof(float) + 2) {

    FloatByteConverter receivedData;

    while (mySerial.read() != startByte) {
      // Keep reading until the start byte is found
    }
    
    // Read the byte array into the float variable
    //mySerial.readBytes((char*)&receivedData, sizeof(double));
    mySerial.readBytes(receivedData.byteArray, sizeof(float));

    // Process the received data
    Serial.print("Received: ");
    Serial.println(receivedData.floatValue); //This only prints, do not copy

    while (mySerial.read() != endByte) {
      // Keep reading until the end byte is found
    }
    delay(1000);
  }
}
