#include <IchibotUltimate4s.h>
#include <SerialControlPersist.h>

#define RXD2 16
#define TXD2 17
SerialControlPersist SerialController;
IchibotUltimate4s ichibot;

float kp = 0;
float kd = 0;
int min_pid = -100;
int max_pid = 100;
int ON = ichibot.ON;
void test()
{
  Serial.println("Changing serial");
  SerialController.setSerial(&Serial1);
}
void testArg(String msg)
{
  Serial.println("yes: " + msg);
}
void setup()
{
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(115200);

  SerialController.setSerial(&Serial1);
  Serial.println("hello there");

  SerialController.registerFLOAT("kp", &kp);
  SerialController.registerFLOAT("kd", &kd);
  SerialController.registerINT("speed", &setting.speed);
  SerialController.registerINT("ON", &ichibot.ON);
  SerialController.registerINT("DEBUG", &ichibot.DEBUG_LOG);
  SerialController.registerINT("min_pid", &min_pid);
  SerialController.registerINT("max_pid", &max_pid);
  SerialController.registerVoidNoArg("test", &test);
  SerialController.registerVoidWithStringArg("testArg", &testArg);

  /* INITIALIZE THE ICHIBOT ULTIMATE 4S LIBRARY */
  ichibot.begin();

  /* SET SENSOR SENSITIVITY (Range 0 - 100) The higher, the more sensitive */
  // ichibot.setSensorSensivity(40);

  /* SET PID VALUES ACCORDING TO YOUR PREFERENCE. This robot uses PD control, so ignore the I values */
  /* Parameters = PID number, Kp, Ki, Kd, Max PWM, Min PWM */
  ichibot.setPID(PID_0, 6, 0, 35, 255, -120);
  ichibot.setPID(PID_1, 8, 0, 40, 255, -120);
  ichibot.setPID(PID_2, 9, 0, 45, 255, -120);
  setting.speed = 80;
  setting.numPID = PID_3;
  ichibot.setPID(PID_3, 10, 0, 15, 130, -120);
  ichibot.setPID(PID_4, 12, 0, 20, 255, -200);
  ichibot.setPID(PID_5, 15, 0, 25, 255, -250);

  /* CHECKPOINT SETTINGS */
  /* Parameters = Checkpoint number, Robot will start running from this index */
  /* For example (CP_1, 8), the robot will be ready to execute index 8 */
  ichibot.setCheckPoint(CP_0, 0);
  ichibot.setCheckPoint(CP_1, 1);
  ichibot.setCheckPoint(CP_2, 2);
  ichibot.setCheckPoint(CP_3, 3);
  ichibot.setCheckPoint(CP_4, 4);
  ichibot.setCheckPoint(CP_5, 5);

  /* SET AT WHICH INDEX THE ROBOT WILL STOP */
  ichibot.StopAtIndex(6);

  /* CREATE PATH PLANNING */
  ichibot.setIndex(0, SENSOR_ALL, MOTION, 120, 120, 1500, BLACK_LINE, 101, 250, PID_3, FAN_OFF, NORMAL);
  ichibot.setIndex(1, SENSOR_LEFT, TURN_LEFT, 100, BLACK_LINE, 101, 200, PID_3, FAN_OFF, NORMAL);
  ichibot.setIndex(2, SENSOR_RIGHT, TURN_RIGHT, 100, BLACK_LINE, 101, 100, PID_3, FAN_OFF, NORMAL);
  ichibot.setIndex(3, SENSOR_LEFT, TURN_LEFT, 100, BLACK_LINE, 101, 300, PID_3, FAN_OFF, NORMAL);
  ichibot.setIndex(4, SENSOR_RIGHT, TURN_RIGHT, 50, BLACK_LINE, 101, 100, PID_3, FAN_OFF, NORMAL);
  ichibot.setIndex(5, SENSOR_LEFT_RIGHT, STRAIGHT, 100, BLACK_LINE, 101, 100, PID_3, FAN_OFF, NORMAL);
  ichibot.setIndex(6, SENSOR_RIGHT, TURN_LEFT, 100, BLACK_LINE, 101, 300, PID_5, FAN_OFF, NORMAL);
  Serial.println("Starting");
}

void loop()
{

  ichibot.testPID();

  if (SerialController.update())
  {
    String key = SerialController.lastKey;
    String value = SerialController.lastValue;
    if (key == "kp" || key == "kd" || key == "min_pid" || key == "max_pid")
    {
      // Serial1.println("--->"+key+"=" + value);
      ichibot.setPID(PID_3, kp, 0, kd, max_pid, min_pid);
    }

    else if (key == "ON")
    {
      ON = ON ? 0 : 1;
      ichibot.ON = ON;
      // Serial1.println("---> ON=" + String(ichibot.ON));
    }
    else if (key == "DEBUG")
    {
      // Serial1.println("---> DEBUG=" + String(ichibot.DEBUG_LOG));
    }
    else if (key == "speed")
    {
      // Serial1.println("---> speed=" + String(setting.speed));
    }
  };
}
