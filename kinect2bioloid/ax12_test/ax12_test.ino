#include <ax12.h>

AX12 motor;
int p, id, mode;

// Gesture Mimicking State
const int numMotors = 6;
char packet[numMotors + 1];
const int readingMultiplier = 10;
int lastReadingSent[numMotors];
const int hysterisisThreshold = 1;
const int motorVelocity = 100;
void setup() {
 
  motor = AX12();

  Serial.begin (57600);  // inicializa el Serial a 115,2 Kb/s
  AX12::init (1000000);   // inicializa los AX12 a 1 Mb/s

  byte detect;
//  byte num = AX12::autoDetect (&detect, 1); // detección de IDs

  //Serial.print (" deteccion: ");
  //Serial.println (num, DEC);

  motor.id = 1;//detect; // asigna las ID detectadas a los motores definidos previamente
  motor.SRL = RETURN_ALL;
  /*
  Serial.print (" ID detectado: ");
  Serial.println (detect, DEC);
  Serial.print (" delay: ");
  AX12info test = motor.readInfo (RETURN_DELAY_TIME);
  Serial.println (test.value, DEC);
  Serial.print (" error lectura: ");
  Serial.println (test.error, DEC);
*/
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
