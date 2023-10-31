#include <Arduino.h>
#include "SerialControlPersist.h"

SerialControlPersist SerialController;
int yahoo = 12;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("BEGIN");
  SerialController.registerINT("yahoo",&yahoo);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (SerialController.update()){
    Serial.println("---> yahoo="+String(yahoo));
  };
  delay(1);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}