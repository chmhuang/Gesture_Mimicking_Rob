#include <ax12.h>

AX12 motor;
int p, id, mode;

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
}


void loop() {
  while(!Serial.available()) {
  }
  char packet[6];
  Serial.readBytesUntil('\n', packet, 6);
  const int readingMultiplier = 20;

  motor.id = 1;
  char reading = packet[0];
  motor.setVel(150);
  motor.setPos(reading * readingMultiplier);
  
  motor.id = 2;
  reading = packet[1];
  motor.setVel(150);
  motor.setPos(reading * readingMultiplier);
  
  motor.id = 3; 
  reading = packet[2];
  motor.setVel(150);
  motor.setPos(reading * readingMultiplier);

  motor.id = 4;
  reading = packet[3];
  motor.setVel(150);
  motor.setPos(reading  * readingMultiplier);
  
  motor.id = 6;
  reading = packet[4];
  motor.setVel(250);
  motor.setPos(reading * readingMultiplier);
  
  // set unused motor to 500
  motor.id = 5;
  motor.setVel(150);
  motor.setPos(500);
}
