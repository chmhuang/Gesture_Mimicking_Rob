#include <ax12.h>

AX12 motor;
int p, id, mode;

// Gesture Mimicking State
const int numMotors = 12;
char packet[numMotors + 1];
const int readingMultiplier = 10;
int lastReadingSent[numMotors];
const int hysterisisThreshold = 1;
const int motorVelocity = 100;
void setup() {
  Serial.begin (57600);

  // AX12 
  motor = AX12();


  AX12::init (1000000);   // inicializa los AX12 a 1 Mb/s

  byte detect;

  motor.id = 1;//detect; // asigna las ID detectadas a los motores definidos previamente
  motor.SRL = RETURN_ALL;

  // Initialize state
  p = 2;
  id = motor.id;
  mode = 1;
  //motor.writeInfo (ID, 5);
  for (int i = 0; i < numMotors; i++) {
    //Initialize all to 50
    lastReadingSent[i] = 50;
  }
}


void loop() {
  if (Serial.available()) {
    char packet[13];
    boolean received = false;
      Serial.readBytesUntil('\n', packet, numMotors);
      for (int i = 0; i < numMotors; i++) {
        char reading = packet[i];
        if (abs(reading - lastReadingSent[i]) > hysterisisThreshold) {
          lastReadingSent[i] = reading;
        }
     }
  }

  for (int i = 0; i < numMotors; i++) {
    motor.id = i+1;
    motor.setVel(motorVelocity);
    motor.setPos(lastReadingSent[i] * readingMultiplier);
  }
}
