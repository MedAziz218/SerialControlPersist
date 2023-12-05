//   Serial.begin(115200);
//   Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
#include <EEPROM.h>
#include <Arduino.h>
#include <robot.h>
#include <SerialControlPersist.h>
#define RXD2 16
#define TXD2 17
SerialControlPersist SerialController;
Robot lampa;
Stream *DebugSerial;
const int EEPROM_SIZE = 1;
int start_flag = 0;

int kp = 0, kd = 0, pid_num = 0, pid_min = 0, pid_max = 0, speed = 0;

void waitForResetButton();
void tunePIDCode();
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

  SerialController.registerINT("kp", &kp);
  SerialController.registerINT("kd", &kd);
  SerialController.registerINT("pid_max", &pid_max);
  SerialController.registerINT("pid_min", &pid_min);
  SerialController.registerINT("pid_num", &pid_num);
  SerialController.registerINT("speed", &speed);

  lampa.setPID(PID_0, 35, 0, 5, 140, -120);
  lampa.setPID(PID_1, 55, 0, 8, 140, -120);
  // lampa.setPID(PID_1, 40, 0, 8, 255, -120);
  lampa.setPID(PID_2, 45, 0, 9, 255, -120);
  lampa.setPID(PID_3, 15, 0, 10, 255, -170);
  lampa.setPID(PID_4, 20, 0, 12, 255, -200);
  lampa.setPID(PID_5, 25, 0, 15, 255, -250);
  lampa.begin();

  waitForResetButton();

  delay(500);
  DebugSerial->println("holaa");
  lampa.setPIDNum(PID_0);
  lampa.setSpeed(120);
  lampa.setLineColor(BLACK_LINE);
  lampa.setState(2);
}

// lampa.followLineUntilSensor(0b100000,0b001100,XOR);
void loop()
{
  if (lampa.ON == 1)
  {
    // code el maquette
    DebugSerial->println("starting");
    lampa.setPIDNum(PID_0);
    lampa.setSpeed(120);
    // lampa.forwardUntilDelayOrEncoder(0, 370, 370);
    // lampa.setMotorUntilDelayOrEncoder(120,120,1000,0,0); // debugggg

    // Accelerate ------;
    lampa.setSpeed(50);
    lampa.forwardUntilDelayOrEncoder(50, 0, 0);
    lampa.setSpeed(75);
    lampa.forwardUntilDelayOrEncoder(50, 0, 0);
    lampa.setSpeed(100);
    lampa.forwardUntilDelayOrEncoder(50, 0, 0);
    lampa.setSpeed(120);
    // -----------------;

    lampa.followLineUntilSensor(SENSOR_RIGHT);
    lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 155, 0); // debugggg
    lampa.setPIDNum(PID_1);
    lampa.followLineUntilDelayOrEncoder(600, 0, 0);
    // lampa.followLineUntilSensor(0b100000,0b001100,XOR);
    lampa.followLineUntilSensor(0b10000000, 0b00011000, XOR);
    lampa.setMotorUntilDelayOrEncoder(-25, -25, 500, 0, 0); // debugggg

    SerialController.update();
    lampa.ON = 0;
    DebugSerial->println("finished");
  }
  else if (lampa.ON == 2)
  { // code ytesti el pid w y5allik tbadlou bel bluetooth
    lampa.testPID();
    if (SerialController.update())
    {
      tunePIDCode();
    }
  }
  else if (lampa.ON == 0)
  { // code ywa9af e robot
    lampa.setMotor(0, 0);
    lampa.testPID();
    lampa.debugCode();
    if (SerialController.update())
    {
      tunePIDCode();
      if (lampa.ON == 2)
      {
        DebugSerial->println("testing Pid");
      }
    }
    delay(10);
  }
}

void tunePIDCode()
{
  if (SerialController.lastKey == "pid_num")
  {
    lampa.setPIDNum(pid_num);
    DebugSerial->println("changed pid_num: " + pid_num);
  }
  if (SerialController.lastKey == "kp")
  {
    lampa.setPID(setting.numPID, kp, 0,
                 listPID[setting.numPID].Kd, listPID[setting.numPID].PMax,
                 listPID[setting.numPID].PMin);
    DebugSerial->println("changed kp: " + kp);
  }
  if (SerialController.lastKey == "kd")
  {
    lampa.setPID(setting.numPID, listPID[setting.numPID].Kp, 0,
                 kd, listPID[setting.numPID].PMax,
                 listPID[setting.numPID].PMin);
    DebugSerial->println("changed kd: " + kd);
  }
  if (SerialController.lastKey == "pid_max")
  {
    lampa.setPID(setting.numPID, listPID[setting.numPID].Kp, 0,
                 listPID[setting.numPID].Kd, pid_max,
                 listPID[setting.numPID].PMin);
    DebugSerial->println("changed pid_max: " + pid_max);
  }
  if (SerialController.lastKey == "pid_min")
  {
    lampa.setPID(setting.numPID, listPID[setting.numPID].Kp, 0,
                 listPID[setting.numPID].Kd, listPID[setting.numPID].PMax,
                 pid_min);
    DebugSerial->println("changed pid_min: " + pid_min);
  }
  if (SerialController.lastKey == "speed")
  {
    lampa.setSpeed(speed);
    DebugSerial->println("changed speed: " + speed);
  }
}
void waitForResetButton()
{
  // --------------------------- REMOVE ME
  // EEPROM.begin(EEPROM_SIZE);
  // EEPROM.write(0, 0);
  // EEPROM.commit();
  // while (1){ delay(100);}
  // --------------------------- REMOVE ME

  EEPROM.begin(EEPROM_SIZE);
  start_flag = EEPROM.read(0);
  EEPROM.write(0, !start_flag);
  EEPROM.commit();
  pinMode(2, OUTPUT);
  DebugSerial->println("start_flag: " + String(start_flag));
  if (!start_flag)
  {
    // digitalWrite(2,HIGH);
    while (1)
    {
      Serial.println("waiting");
      SerialController.update();
      lampa.debugCode();
      delay(100);
    }
  }
}