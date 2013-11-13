/*
  Blink
 Turns on an LED on for one second, then off for one second, repeatedly.
 
 This example code is in the public domain.
 */

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);  
}

// the loop routine runs over and over again forever:
void loop(){
  if (Serial.available()>0) {
    byte input=Serial.read();
    if(input == '1'){
      digitalWrite(led, HIGH);
    }
    else{
      digitalWrite(led, LOW);
    }
  }
}

