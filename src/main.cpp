#include <Arduino.h>
#include "SerialControlPersist.h"
#define RXD2 16
#define TXD2 17
SerialControlPersist SerialController;
int yahoo = 12;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial1.println("hello there");
  Serial.println("BEGIN");
  

  SerialController.registerINT("yahoo",&yahoo);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (SerialController.update()){

    Serial1.println("---> yahoo="+String(yahoo));
    Serial.println("---> yahoo="+String(yahoo));
  };
  // if (Serial.available()){
  //   Serial1.write(Serial.read());
  // }
  // if (Serial1.available()){
  //   Serial.write(Serial1.read());
  // }

  delay(1);
}
