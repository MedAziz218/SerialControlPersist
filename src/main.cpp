
//   Serial.begin(115200);
//   Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);


#include <Arduino.h>
#include <robot.h>
#include <SerialControlPersist.h>
#define RXD2 16
#define TXD2 17
SerialControlPersist SerialController;
Robot lampa;
int ON = 0;
void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  

  SerialController.setSerial(&Serial1);
  SerialController.registerINT("ON",&ON);
  SerialController.registerINT("GetPid",&lampa.DEBUG_Pid);
  
  lampa.begin();
  lampa.setDebugSerial(&Serial);
  lampa.setPID(PID_0, 35, 0, 5, 140, -100);
  lampa.setPID(PID_1, 40, 0, 8, 255, -120);
  lampa.setPID(PID_2, 45, 0, 9, 255, -120);
  lampa.setPID(PID_3, 15, 0, 10, 255, -170);
  lampa.setPID(PID_4, 20, 0, 12, 255, -200);
  lampa.setPID(PID_5, 25, 0, 15, 255, -250);
  lampa.setPIDNum(PID_0);
  lampa.setSpeed(100);
  lampa.setLineColor(BLACK_LINE);
  
  

  // lampa.forwardUntilSensor(SENSOR_ALL_HIT);
  
  
  delay(1000);
  Serial1.println("holaa");
  lampa.forwardUntilDelayOrEncoder(0,80,100);
  Serial1.println("done");

}

void loop()
{
  SerialController.update();
  lampa.testPID();
}