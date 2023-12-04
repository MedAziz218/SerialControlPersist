
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
Stream* DebugSerial;
void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  DebugSerial = &Serial1;
  SerialController.setSerial(DebugSerial);
  lampa.setDebugSerial(DebugSerial);

  SerialController.registerINT("ON", &lampa.ON);
  SerialController.registerINT("GetPid", &lampa.DEBUG_Pid);
  SerialController.registerINT("GetEnc", &lampa.DEBUG_Encoders);
  SerialController.registerINT("ResetEnc", &lampa.DEBUG_ResetEnc);

  lampa.begin();

  lampa.setPID(PID_0, 35, 0, 5, 140, -120);
  lampa.setPID(PID_1, 55, 0, 8, 140, -120);
  // lampa.setPID(PID_1, 40, 0, 8, 255, -120);
  lampa.setPID(PID_2, 45, 0, 9, 255, -120);
  lampa.setPID(PID_3, 15, 0, 10, 255, -170);
  lampa.setPID(PID_4, 20, 0, 12, 255, -200);
  lampa.setPID(PID_5, 25, 0, 15, 255, -250);
  lampa.setPIDNum(PID_0);
  lampa.setSpeed(150);
  lampa.setLineColor(BLACK_LINE);


  delay(2000);
  DebugSerial->println("holaa");
  // while (!lampa.ON)
  // {
  //   SerialController.update();
  // }
  // Serial.println("starting");
  // lampa.forwardUntilSensor(SENSOR_ALL_HIT);
  // lampa.setMotor(0,0);
  // Serial.println("done");
  lampa.setState(1);

}

    // lampa.followLineUntilSensor(0b100000,0b001100,XOR);
void loop()
{
  if (lampa.ON == 1)
  {
    DebugSerial->println("starting");
    lampa.setPIDNum(PID_0);
    lampa.setSpeed(120);
    // lampa.forwardUntilDelayOrEncoder(0, 370, 370);
    // lampa.setMotorUntilDelayOrEncoder(120,120,1000,0,0); // debugggg
    lampa.followLineUntilSensor(SENSOR_RIGHT);
    lampa.setMotorUntilDelayOrEncoder(120,-25,0,155,0); // debugggg
    lampa.setPIDNum(PID_1);
    lampa.followLineUntilDelayOrEncoder(600,0,0);
    lampa.followLineUntilSensor(0b100000,0b001100,XOR);
    lampa.setMotorUntilDelayOrEncoder(-25,-25,500,0,0); // debugggg



    
    
    SerialController.update();
    lampa.ON = 0 ;
    DebugSerial->println("finished");
  }
  else if (lampa.ON == 2)
  {
    lampa.testPID();
    SerialController.update();
  }
  else if (lampa.ON == 0)
  {
    lampa.setMotor(0, 0);
    lampa.testPID();
    lampa.debugLoop();
    if (SerialController.update())
    {
        if (lampa.ON==2) {DebugSerial->println("testing Pid");}
    }
    delay(10);
  }
}