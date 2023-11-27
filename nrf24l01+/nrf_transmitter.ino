#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = {0xe1, 0xf0, 0xf0, 0xf0, 0xf0}; //"00001"

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  double origin[2] = { 29.581607955892935, -98.6202546046463 };
  radio.write(&origin, sizeof(origin));
  delay(1000);
}
