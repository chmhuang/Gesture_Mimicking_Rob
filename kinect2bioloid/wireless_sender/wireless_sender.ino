#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Based on example code and uses library by J. Coliz <maniacbug@ymail.com>
// http://maniacbug.wordpress.com/2011/11/02/getting-started-rf24/

// Set up nRF24L01 radio on SPI bus plus pins 9 & 8
RF24 radio(9,8);
// Radio pipe address for nodes to communicate.
const uint64_t pipe = 0xF0F0F0F0E1LL;

void setup() {
  Serial.begin(57600);
  Serial.println("ready");
  radio.begin();
  radio.openReadingPipe(0, pipe);
  radio.startListening();
}

void loop() {
  if (Serial.available()) {
    Serial.print("Sending message from Serial...");
    char packet[14];
    int bytesRead = Serial.readBytesUntil(0, packet, 14);
    radio.stopListening();
    radio.openWritingPipe(pipe);
    bool delivered = radio.write(packet, 14);
    radio.openReadingPipe(0, pipe);
    radio.startListening();
    if (delivered) {
      Serial.println("Success.");
    } else {
      Serial.println("Fail.");
    }
  }
  delay(1000);
}
