
#include <Servo.h>

Servo myservo; 

int potpin = 0; 
int val;   

void setup() {
  myservo.attach(9);// conectar el pin a el pad de senyal del variador juntament amb el gnd al pad del costat
  myservo.writeMicroseconds(1000);
  delay(5000);
}

void loop() {
  val = analogRead(potpin);           
  val = map(val, 0, 1023, 1000, 2000);     
  myservo.writeMicroseconds(val);         

}

