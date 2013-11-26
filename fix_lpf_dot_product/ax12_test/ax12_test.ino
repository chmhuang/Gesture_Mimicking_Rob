#include <ax12.h>

AX12 motor;
int p, id, mode;

void setup() {
 
  motor = AX12();

  Serial.begin (9600);  // inicializa el Serial a 115,2 Kb/s
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
  char packet[5];
  Serial.readBytesUntil('\n', packet, 5);
    motor.id = 1;
    char reading = packet[0];
    motor.setVel (150);
    motor.setPos (reading * 10);
    
    motor.id = 2;
    reading = packet[1];
    motor.setVel (150);
    motor.setPos (reading * 10);
    
    motor.id = 4;
    reading = packet[2];
    motor.setVel(150);
    motor.setPos(reading * 10);
    /*
    motor.id = 2;
    motor.setPos(800);
    motor.id = 3; 
    motor.setPos (500);
    motor.id = 4; 
    motor.setPos (500);
    motor.id = 5; 
    motor.setPos (500);
    */

/*
    if (Serial.available() > 0) {
      int tmp = Serial.read() - 48;
      if (tmp >= 0 && tmp <= 9) {
        Serial.print("I received: ");
        Serial.println(tmp);
        if (tmp == 0&& false) {
          mode = 0;
          Serial.println("mode changed to 0 - id");
        } else if (tmp == 9 && false) {
          mode = 1;
          Serial.println("mode changed to 1 - position");
        } else {
          if (mode == 0) {
            id = 1;//0+tmp;
            motor.id = id;
            Serial.print("new id: ");
            Serial.println(id);
          } else {
            p = tmp;
            motor.setVel (300);//random (100, 300));
            motor.setPos (p*100);//random (200, 800));
            Serial.print("new position: ");
            Serial.print(p*100);
            Serial.print(" to id: ");
            Serial.println(id);
          }
        }
      }
    }
    
     //5delay (100);
    /*
    int pos = motor.getPos(); 
    int vel = motor.getSpeed(); 
    int load = motor.getLoad();
    
    Serial.println (" ");
    Serial.print (" posicion: ");
    Serial.println (pos, DEC);
    Serial.print (" velocidad: ");
    Serial.println (vel, DEC);
    Serial.print (" carga: ");
    Serial.println (load, DEC);
    */    
    //delay (1100);
  
}
