#include <Wire.h>  // Library de i2c
#define SLAVE_ADDRESS 0x40 // adreça esclau
volatile short valor; 
 unsigned char var;
 
void setup() {
  Wire.begin(SLAVE_ADDRESS); // li assignem l'adreça 0x40 
  Wire.onRequest(pregunta); // cuan demanem informacio a l'esclau fara la funcio pregunta
  Wire.onReceive(accio); // ordenem a l'esclau a executar la funcio accio
  pinMode(3,INPUT);//pin de prova (boto)
  pinMode(4,OUTPUT);//pin de prova (led)
}

void loop() {
if (var==1){
  digitalWrite(4,HIGH);
  }
if (var==0){
  digitalWrite(4,LOW);
}
}
void pregunta(){//indiquem l'estat del interruptor perque la rasp ho gestioni.
 valor=digitalRead(3);
 Wire.write(byte(valor));
}
void accio(){//llegim si volem encendre o apagar el led, depenent si hem rebut un 0 o un 1 de la rasp.
  while (Wire.available())
  {
    var=Wire.read();
    }
  
  }
