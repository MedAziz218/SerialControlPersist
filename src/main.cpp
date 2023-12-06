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
void debugPause(int dt)
{
  lampa.setMotorUntilDelayOrEncoder(-25, -25, dt, 0, 0);
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
  lampa.setPIDNum(PID_1);
  lampa.setSpeed(180);
  lampa.setLineColor(BLACK_LINE);
  lampa.setState(1);
}

// lampa.followLineUntilSensor(0b100000,0b001100,XOR);
void loop()
{
  if (lampa.ON == 1)
  {
    for (int i = 0; i < 3; i++)
    {
      if (1)
      {

        // code el maquette
        DebugSerial->println("starting");
        lampa.setPIDNum(PID_0);
        lampa.setSpeed(120);
        lampa.setLineColor(BLACK_LINE);
        if (i == 0 || i==2)
        {
          if (i==0){

          lampa.setPIDNum(PID_0);
          // Accelerate ------;
          lampa.setSpeed(50);
          lampa.forwardUntilDelayOrEncoder(20, 0, 0);
          lampa.setSpeed(75);
          lampa.forwardUntilDelayOrEncoder(20, 0, 0);
          lampa.setSpeed(100);
          lampa.forwardUntilDelayOrEncoder(20, 0, 0);
          lampa.setSpeed(120);
          lampa.forwardUntilDelayOrEncoder(0, 226, 226);

          // -----------------;

          }
          lampa.followLineUntilSensor(SENSOR_RIGHT);
          lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 155, 0); // awal dora right
          lampa.followLineUntilDelayOrEncoder(125, 0, 0);         // cooldown
          // debugPause(200);
          lampa.followLineUntilSensor(SENSOR_RIGHT_CENTER); // detecti moftara9
          // debugPause(1000);
          lampa.setMotorUntilDelayOrEncoder(-25, 120, 0, 0, 90); // dour el moftara9 encL:3 encR:95
          lampa.followLineUntilDelayOrEncoder(125, 0, 0);        // cooldown
        } else if (i==1){
          lampa.setPIDNum(PID_0);
          lampa.setSpeed(120);
          lampa.followLineUntilDelayOrEncoder(200,0,0);
          lampa.followLineUntilSensor(SENSOR_RIGHT);
          lampa.followLineUntilDelayOrEncoder(0,150,150);
          // debugPause(1000);
          lampa.followLineUntilSensor(SENSOR_LEFT);

        }
        lampa.followLineUntilSensor(SENSOR_RIGHT); // dora el m3aftaa
        // lampa.followLineUntilSensor(0b10000000,0b00011000,XOR); //

        lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 300, 0); // turn right
        lampa.setMotorUntilDelayOrEncoder(120, 120, 0, 30, 30); // 9addam chwaya
        // lampa.setMotorUntilDelayOrEncoder(-25, -25, 20, 0, 0); // debugggg
        lampa.setMotorUntilDelayOrEncoder(-25, 120, 0, 0, 469);     // turn left
        lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL); // centri ro7k
        lampa.followLineUntilDelayOrEncoder(100, 0, 0);             // cooldown
        // lampa.followLineUntilDelayOrEncoder(1000, 0, 0);
        lampa.setPIDNum(PID_2);
        // lampa.followLineUntilSensor(0b10000000, 0b00011000, XOR); // detecti carro jary
        lampa.followLineUntilDelayOrEncoder(200,0,0);
        // debugPause(500);
        lampa.setPIDNum(PID_0);

        lampa.followLineUntilDelayOrEncoder(0, 80, 80); // centri ro7k
        lampa.setSpeed(200);
        lampa.setPIDNum(PID_1);
        // ejryyyyy fantar
        lampa.followLineUntilDelayOrEncoder(0, 450, 450); // encL:744 encR:771
        // raja3 low speed
        lampa.setSpeed(120);
        lampa.setPIDNum(PID_0);
        // lampa.followLineUntilSensor(0b10000000, 0b00111000, XOR); // detecti carro w9ouf
        // encL:252 encR:1
        //  encL:2 encR:469

        // detecti dora m3afta 2
        lampa.followLineUntilSensor(0b10000000, 0b00111000, XOR);
        lampa.forwardUntilDelayOrEncoder(0, 90, 90);
        debugPause(50);
        lampa.setMotorUntilDelayOrEncoder(-25, 120, 0, 0, 375);
        // debugPause(50);
        lampa.forwardUntilDelayOrEncoder(0, 70, 70);
        // debugPause(50);
        lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 550, 0);
        // debugPause(50);
        //  encL:94 encR:110
        //  encL:2 encR:260
        //  encL:71 encR:100
        //  encL:552 encR:0

        // kamalna dora lm3afta 2
      }
      if (1)
      {
        // ejri chwaya
        lampa.setPIDNum(PID_0);
        lampa.setSpeed(150);
        lampa.followLineUntilDelayOrEncoder(0, 160, 160);
        lampa.followLineUntilSensor(SENSOR_RIGHT);

        // dour right
        lampa.setMotorUntilDelayOrEncoder(120, -25, 0, 155, 0); // awal dora right
        lampa.setPIDNum(PID_1);
        lampa.setSpeed(180);
        lampa.followLineUntilDelayOrEncoder(0, 580, 580); // bdina theni dora right
        lampa.setPIDNum(PID_0);
        lampa.setSpeed(120);
        lampa.followLineUntilDelayOrEncoder(0, 550, 0); // kamalna theni dora right
        lampa.setPIDNum(PID_1);
        lampa.setSpeed(180);
        lampa.followLineUntilDelayOrEncoder(0, 400, 0); // kamalna partie mestwia

        lampa.setPIDNum(PID_5);
        lampa.setSpeed(120);
        lampa.setPIDNum(PID_2);
        lampa.followLineUntilDelayOrEncoder(2600, 1500, 0); // kamalna nos dora s8ayra

        lampa.setPIDNum(PID_0);
        lampa.setSpeed(120);
        lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL); // partie mestwia yesr twila

        //--------------------------------------- REMOVE ME
        unsigned int encL = get_encL(), encR = get_encR();
        unsigned int ms = millis();
        //--------------------------------------- REMOVE ME

        //--------------------------------------- REMOVE ME
        encL = get_encL() - encL;
        encR = get_encR() - encR;
        DebugSerial->println("encL: " + String(encL) + " encR: " + String(encR) + "ms: " + String(millis() - ms));
        //--------------------------------------- REMOVE ME

        // debugPause(1000);
        lampa.setPIDNum(PID_3);
        lampa.setSpeed(255);
        lampa.followLineUntilDelayOrEncoder(0, 500, 500); // kamalna partie mestwia
      }
      lampa.setPIDNum(PID_0);
      lampa.setSpeed(150);
      lampa.followLineUntilSensor(SENSOR_LEFT); // kamalna partie mestwia (carro loul)

      lampa.setPIDNum(PID_0);
      lampa.setSpeed(120);
      lampa.followLineUntilDelayOrEncoder(200, 0, 0);
      lampa.followLineUntilSensor(SENSOR_LEFT); // kamalna partie mestwia (carro theni)
      lampa.followLineUntilDelayOrEncoder(200, 0, 0);
      lampa.followLineUntilSensor(SENSOR_LEFT); // carro theleth
      lampa.followLineUntilDelayOrEncoder(200, 0, 0);
      lampa.followLineUntilSensor(SENSOR_LEFT); // carro raba3 ba3d dora
      lampa.followLineUntilDelayOrEncoder(200, 0, 0);
      lampa.followLineUntilSensor(SENSOR_RIGHT_CENTER);       // carro 5ames ba3d dora
      if (i==0){ //PITSTOP
      lampa.setMotorUntilDelayOrEncoder(120, -30, 0, 240, 0); // d5alna pitStop
      // lampa.followLineUntilSensor(0b00011000, 0b00000000, EQUAL);
      lampa.followLineUntilSensor(SENSOR_EMPTY);
      lampa.forwardUntilDelayOrEncoder(0,50,50);
      lampa.setMotorUntilDelayOrEncoder(-30,-30,820,0,0);
      lampa.setMotorUntilDelayOrEncoder(-25,-25,1000,0,0);
      lampa.forwardUntilSensor(SENSOR_ALL);
      lampa.setPIDNum(PID_2);
      lampa.setSpeed(80);
      
      
      }
      else if (i==1 || i==2){
        // debugPause(1000);
        // encL:92 encR:80
        // encL:0 encR:157
        // encL:113 encR:1
        // --------------------
        // encL:66 encR:69
        // encL:58 encR:76
        lampa.setPIDNum(PID_2);
        lampa.setSpeed(80);
        lampa.forwardUntilDelayOrEncoder(0,64,64);
        debugPause(25);

        lampa.followLineUntilDelayOrEncoder(2000,0,0);
        // debugPause(500);
        // lampa.followLineUntilSensor(SENSOR_RIGHT);
        lampa.followLineUntilSensor(0b00011000,0b00000000,EQUAL);

        // lampa.setMotorUntilDelayOrEncoder(-120,120,0,58,58);
        
        
        // lampa.setMotorUntilDelayOrEncoder(120,120,0,80,80);
        // debugPause(25);
        // lampa.setMotorUntilDelayOrEncoder(120,-25,0,235,0);
        // debugPause(5000);

        // lampa.followLineUntilSensor(SENSOR_RIGHT);

        

        // debugPause(50);

      }
    }
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
  pinMode(2, OUTPUT);
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