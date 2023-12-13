//   Serial.begin(115200);
//   Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
#include <EEPROM.h>
#include <Arduino.h>
#include <robot.h>
#include <SerialControlPersist.h>
#define RXD2 16
#define TXD2 17

// lampa.setPID(PID_0, 30, 0, 5, 140, -120);
// lampa.setPID(PID_1, 20, 0, 5, 200, -100);
// // lampa.setPID(PID_1, 40, 0, 8, 255, -120);
// lampa.setPID(PID_2, 25, 0, 5, 100, -100);
// lampa.setPID(PID_3, 20, 0, 10, 255, -120);
// lampa.setPID(PID_4, 20, 0, 12, 255, -200);
// lampa.setPID(PID_5, 35, 0, 5, 140, -140);
#define pid0 PID_0, 35, 0, 5, 140, -120
#define pid1 PID_0, 25, 0, 5, 200, -100
#define pid2 PID_0, 25, 0, 5, 100, -100
#define pid3 PID_0, 20, 0, 10, 255, -120
#define pid4 PID_0, 20, 0, 12, 255, -200
#define pid5 PID_0, 25, 0, 5, 140, -140

SerialControlPersist SerialController;
Robot lampa;
Stream *DebugSerial;
const int EEPROM_SIZE = 1;
int start_flag = 0;

int kp = 0, kd = 0, pid_num = 0, pid_min = 0, pid_max = 0, speed = 0;

void waitForResetButton();
void tunePIDCode();
void debugPause(int dt)
{
  lampa.setMotorUntilDelayOrEncoder(-30, -30, dt, 0, 0);
}
unsigned int debugTimer = 0;
void debugLog(String log)
{
  float dt = float(millis() - debugTimer);
  dt = dt / 1000;
  DebugSerial->println(String(dt, 2) + "-> " + log);
}

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

  lampa.setPID(PID_0, 30, 0, 5, 140, -120);
  lampa.setPID(PID_1, 20, 0, 5, 200, -100);
  // lampa.setPID(PID_1, 40, 0, 8, 255, -120);
  lampa.setPID(PID_2, 25, 0, 5, 100, -100);
  lampa.setPID(PID_3, 20, 0, 10, 255, -120);
  lampa.setPID(PID_4, 20, 0, 12, 255, -200);
  lampa.setPID(PID_5, 35, 0, 5, 140, -140);
  lampa.begin();

  waitForResetButton();

  delay(500);
  DebugSerial->println("holaa");
  lampa.setPID(pid2); // lampa.setPIDNum(PID_0);
  lampa.setSpeed(80);
  lampa.setLineColor(BLACK_LINE);
  lampa.setState(1);
}
// clang-format off
int i=1;
int lap=1;
// clang-format on
void loop()
{
  if (lampa.ON == 2)
  {

    lampa.testPID();
  }
  if (lampa.ON == 1)
  {

    debugTimer = millis();
    debugLog("start point");
    lampa.setPIDNum(PID_0);
    // -- -- -- -- -- -- -- -- -- -i = 1;
    if (i <= 1)
    {
      lampa.setPID(pid0); // lampa.setPIDNum(PID_0);
      lampa.setSpeed(120);
      if (lap == 1)
      {
        // Accelerate ------;

        lampa.setSpeed(50);
        lampa.forwardUntilDelayOrEncoder(20, 0, 0);
        lampa.setSpeed(75);
        lampa.forwardUntilDelayOrEncoder(20, 0, 0);
        lampa.setSpeed(100);
        lampa.forwardUntilDelayOrEncoder(20, 0, 0);
        lampa.setSpeed(120);
        lampa.forwardUntilDelayOrEncoder(0, 226, 226);

        // -----------------i=1;
      }

      if (lap == 1 || lap == 2)
      {

        lampa.setPID(pid0); // lampa.setPIDNum(PID_0);
        lampa.setSpeed(120);
        lampa.followLineUntilSensor(SENSOR_RIGHT);
        lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 155, 0); // awal dora right

        lampa.followLineUntilDelayOrEncoder(125, 0, 0); // cooldown
        debugLog("kamalt awal dora right");

        lampa.followLineUntilSensor(SENSOR_RIGHT_CENTER); // detecti moftara9

        lampa.setMotorUntilDelayOrEncoder(-25, 120, 0, 0, 90); // dour el moftara9 encL:3 encR:95
        lampa.followLineUntilDelayOrEncoder(0, 226, 226);      // cooldown
        debugLog("kamalt moftara9");
      }
    }

    if (i <= 2)
    {
      lampa.followLineUntilSensor(SENSOR_RIGHT); // dora el m3aftaa
      debugLog("d5alna dora lm3afta 1");

      // -----------------i=2;

      // lampa.followLineUntilSensor(0b10000000,0b00011000,XOR); //

      lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 300, 0); // turn right
      lampa.setMotorUntilDelayOrEncoder(120, 120, 0, 30, 30); // 9addam chwaya
      // lampa.setMotorUntilDelayOrEncoder(-25, -25, 20, 0, 0); // debugggg
      lampa.setMotorUntilDelayOrEncoder(-25, 120, 0, 0, 469);     // turn left
      lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL); // centri ro7k
      lampa.followLineUntilDelayOrEncoder(100, 0, 0);             // cooldown
      // lampa.followLineUntilDelayOrEncoder(1000, 0, 0);
      debugLog("kamalna dora m3afta 1");
    }

    // -----------------i=3;
    if (i <= 3)
    { // lampa.followLineUntilSensor(0b10000000, 0b00011000, XOR); // detecti carro jary
      // lampa.setPIDNum(PID_2);
      lampa.setPID(pid2);
      lampa.followLineUntilDelayOrEncoder(200, 226, 226);
      lampa.followLineUntilDelayOrEncoder(0, 80, 80); // centri ro7k

      debugLog("ejrii");

      // lampa.setSpeed(200);
      // lampa.setPID(pid1); // lampa.setPIDNum(PID_1);

      // ejryyyyy fantar
      // lampa.followLineUntilDelayOrEncoder(0, 250, 250); // encL:744 encR:771
      lampa.setSpeed(180);
      lampa.setPID(pid1);                               // lampa.setPIDNum(PID_1);
      lampa.followLineUntilDelayOrEncoder(0, 250, 250); // encL:744 encR:771

      // raja3 low speed
      debugLog("raja3 low speed");
      lampa.setSpeed(120);
      lampa.setPID(pid0); // lampa.setPIDNum(PID_0);
    }
    // -----------------i=4;
    if (i <= 4)
    {
      // lampa.followLineUntilSensor(0b10000000, 0b00111000, XOR); // detecti carro w9ouf
      // encL:252 encR:1
      //  encL:2 encR:469

      // detecti dora m3afta 2
      lampa.followLineUntilSensor(0b10000000, 0b00111000, XOR);
      debugLog("d5alna dora m3afta 2");
      lampa.forwardUntilDelayOrEncoder(0, 90, 90);
      debugPause(50);
      lampa.setMotorUntilDelayOrEncoder(-25, 120, 0, 0, 375);
      // debugPause(50);
      lampa.forwardUntilDelayOrEncoder(0, 70, 70);
      // debugPause(50);
      lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 550, 0);
      debugLog("kamalna dora m3afta 2 ");
    }

    // -----------------i=5;
    if (i <= 5)
    {
      lampa.setPID(pid0); // lampa.setPIDNum(PID_0);
      lampa.setSpeed(150);
      lampa.followLineUntilDelayOrEncoder(0, 160, 160);
      lampa.followLineUntilSensor(SENSOR_RIGHT);

      debugLog("ta3wija 3al imin 1");
      lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 155, 0); // awal dora right
      lampa.searchAndReturnToLine(-3);

      debugLog("ejri chwaya lenna"); // encL:580 encR:580
      lampa.setPID(0, 35, 0, 5, 180, -130);
      lampa.setSpeed(180);
      lampa.followLineUntilDelayOrEncoder(0, 580, 580);
      // {
      //   lampa.setPID(pid3);
      //   lampa.setSpeed(230);
      //   lampa.followLineUntilDelayOrEncoder(0, 480, 480); // bdina theni dora right
      //   lampa.setPID(pid1);
      //   lampa.setSpeed(180);
      //   lampa.followLineUntilDelayOrEncoder(0, 100, 100); // bdina theni dora right
      // }
      debugLog("ta3wija 3al imin 2");
      lampa.setPID(pid0); // lampa.setPIDNum(PID_0);
      lampa.setSpeed(120);
      lampa.shiftCenter(+1);
      lampa.followLineUntilDelayOrEncoder(0, 550, 0); // kamalna theni dora right
      lampa.shiftCenter(0);
      lampa.searchAndReturnToLine(-3);

      lampa.setPID(pid1); // lampa.setPIDNum(PID_1);
      lampa.setSpeed(180);
      lampa.followLineUntilDelayOrEncoder(0, 400, 400); // kamalna partie mestwia
      debugLog("kamalna partie mestwia 1");
    }
    // ------------------i=6;
    if (i <= 6)
    {

      lampa.setPID(pid0);
      lampa.setSpeed(120);
      lampa.followLineUntilDelayOrEncoder(0, 200, 0);
      debugPause(50);
      lampa.setPID(pid2);
      lampa.setSpeed(80);
      // ----------------------------------||
      int encL = get_encL(), encR = get_encR(), ms = millis();
      lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);
      encL = get_encL() - encL;
      encR = get_encR() - encR;
      ms = millis() - ms;
      lampa.shiftCenter(+1);
      lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);
      lampa.followLineUntilDelayOrEncoder(max(0, 2600 - ms), max(0, 1120 - encL), 0);
      lampa.shiftCenter(0);
      lampa.searchAndReturnToLine(0);
      // ----------------------------------||

      debugLog("kamalna nos dora s8ayra");
      debugLog("ms: " + String(millis() - ms) + " encL: " + String(get_encL() - encL) + " encR: " + String(get_encR() - encR));
    }

    // ------------------i=7;
    if (i <= 7)
    {
      lampa.setPID(pid2);
      lampa.setSpeed(80);
      lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);
      lampa.setPID(pid3);
      lampa.setSpeed(200);
      lampa.followLineUntilDelayOrEncoder(0, 500, 500); // kamalna partie mestwia
      lampa.searchAndReturnToLine(0);
    }
    // ------------------i=8;
    if (i <= 8)
    {
      lampa.setPID(pid0);
      lampa.setSpeed(120);
      lampa.followLineUntilSensor(SENSOR_LEFT); // ndourou feddora (carro theni)
      lampa.shiftCenter(+1);
      lampa.followLineUntilDelayOrEncoder(200, 125, 125);
      lampa.shiftCenter(0);
      lampa.searchAndReturnToLine(-3);

      lampa.setPID(pid0);
      lampa.setSpeed(120);
      lampa.followLineUntilSensor(SENSOR_LEFT); // kamalna partie mestwia (carro loul)

      lampa.followLineUntilDelayOrEncoder(200, 700, 500);
      lampa.followLineUntilSensor(SENSOR_RIGHT);
    }
    // ------------------i=9;
    if (i <= 9)
    {
      if (lap == 2)
      {
        lampa.forwardUntilDelayOrEncoder(0, 50, 50);
        lampa.setMotorUntilDelayOrEncoder(120, -30, 0, 240, 0); // d5alna pitStop
        lampa.setPID(pid2);
        lampa.setSpeed(80);
        lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);
        lampa.followLineUntilSensor(0b11111111, 0b00000000, EQUAL);
        lampa.forwardUntilDelayOrEncoder(0, 125, 125);
        debugPause(1800);
        lampa.forwardUntilDelayOrEncoder(0, 125, 125);
        lampa.setPID(pid2);
        lampa.setSpeed(80);
        // ----------------------------------||
        int encL = get_encL(), encR = get_encR();
        lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);
        encL = get_encL() - encL;
        encR = get_encR() - encR;
        lampa.shiftCenter(-2);
        lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);
        lampa.followLineUntilDelayOrEncoder(0, max(0, 310 - encL), max(0, 310 - encR));
        lampa.shiftCenter(0);
        lampa.searchAndReturnToLine(0);
        // ----------------------------------||

        lampa.followLineUntilDelayOrEncoder(0, 120, 120);
      }
      if (lap == 1 || lap == 3)
      {
        lampa.setPID(pid2);
        lampa.setSpeed(80);
        lampa.forwardUntilDelayOrEncoder(0, 64, 64);
        debugPause(50);

        lampa.setMotorUntilDelayOrEncoder(-120, 120, 0, 64, 64);
        lampa.searchAndReturnToLine(0);
        debugPause(100);

        // ----------------------------------||
        int encL = get_encL(), encR = get_encR(), ms = millis();
        lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);
        encL = get_encL() - encL;
        encR = get_encR() - encR;
        ms = millis() - ms;
        lampa.shiftCenter(+2);
        lampa.followLineUntilDelayOrEncoder(max(0, 2000 - ms), 0, 0);
        lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);

        lampa.shiftCenter(+1);
        // ----------------------------------||
        encL = get_encL() - encL;
        encR = get_encR() - encR;
        lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);
        lampa.setPID(pid0);
        lampa.setSpeed(100);
        lampa.forwardUntilDelayOrEncoder(0, max(0, 400 - encL), max(0, 400 - encR));
        // lampa.followLineUntilSensor(0b00011000,0b00000000,EQUAL);
        // debugPause(100);
        // lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);

        // lampa.followLineUntilSensor(SENSOR_RIGHT);
        // lampa.followLineUntilSensor(0b00011000,0b00000000,EQUAL);
      }
    }
    if (lap == 3)
    {
      lampa.STOP();
      debugLog("STOP");
      debugPause(500);
    }
    else
    {
      lap++;
      i = 1;
    };
  }
  else
  {
    if (SerialController.update())
    {
      lampa.debugCode();
      tunePIDCode();
    }
    delay(10);
  }
}

// lampa.followLineUntilSensor(0b100000,0b001100,XOR);

void tunePIDCode()
{
  // DebugSerial->println("tune pid");

  if (SerialController.lastKey == "pid_num")
  {
    lampa.setPIDNum(pid_num);
    DebugSerial->println("changed pid_num: " + String(pid_num));
  }
  if (SerialController.lastKey == "kp")
  {
    lampa.setPID(setting.numPID, kp, 0,
                 listPID[setting.numPID].Kd, listPID[setting.numPID].PMax,
                 listPID[setting.numPID].PMin);
    DebugSerial->println("changed kp: " + String(kp));
  }
  if (SerialController.lastKey == "kd")
  {
    lampa.setPID(setting.numPID, listPID[setting.numPID].Kp, 0,
                 kd, listPID[setting.numPID].PMax,
                 listPID[setting.numPID].PMin);
    DebugSerial->println("changed kd: " + String(kd));
  }
  if (SerialController.lastKey == "pid_max")
  {
    lampa.setPID(setting.numPID, listPID[setting.numPID].Kp, 0,
                 listPID[setting.numPID].Kd, pid_max,
                 listPID[setting.numPID].PMin);
    DebugSerial->println("changed pid_max: " + String(pid_max));
  }
  if (SerialController.lastKey == "pid_min")
  {
    lampa.setPID(setting.numPID, listPID[setting.numPID].Kp, 0,
                 listPID[setting.numPID].Kd, listPID[setting.numPID].PMax,
                 pid_min);
    DebugSerial->println("changed pid_min: " + String(pid_min));
  }
  if (SerialController.lastKey == "speed")
  {
    lampa.setSpeed(speed);
    DebugSerial->println("changed speed: " + String(speed));
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
  DebugSerial->println("start_flag: " + String(start_flag));
  if (!start_flag)
  {
    // digitalWrite(2,HIGH);
    int lastTime = millis();
    while (1)
    {
      if (millis() - lastTime > 10)
      {
        Serial.println("waiting");
        lastTime = millis();
      }
      if (SerialController.update())
      {
        lampa.debugCode();
      }
    }
  }
}