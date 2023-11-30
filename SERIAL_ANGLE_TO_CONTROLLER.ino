SoftwareSerial mySerial(0, 1); // Define SoftwareSerial object with Tx and Rx pins

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600); // Set the baud rate to 9600 (same as sender Arduino)
}

void loop() {
  if (mySerial.available() >= sizeof(float)) {
    float receivedData;

    // Read the byte array into the float variable
    mySerial.readBytes((char*)&receivedData, sizeof(float));

    // Process the received data
    Serial.print("Received: ");
    Serial.println(receivedData); //This only prints, do not copy
  }
}
