#include <IchibotUltimate4s.h>
IchibotUltimate4s ichibot;

void setup() {
  /* INITIALIZE THE ICHIBOT ULTIMATE 4S LIBRARY */
  ichibot.begin();

  /* SET SENSOR SENSITIVITY (Range 0 - 100) The higher, the more sensitive */
  // ichibot.setSensorSensivity(40);

  /* SET PID VALUES ACCORDING TO YOUR PREFERENCE. This robot uses PD control, so ignore the I values */
  /* Parameters = PID number, Kp, Ki, Kd, Max PWM, Min PWM */
  ichibot.setPID(PID_0,  6, 0, 35, 255, -120);
  ichibot.setPID(PID_1, 8, 0, 40, 255, -120);
  ichibot.setPID(PID_2, 9, 0, 45, 255, -120);
  ichibot.setPID(PID_3, 10, 0, 15, 130, 0);
  ichibot.setPID(PID_4, 12, 0,  20, 255, -200);
  ichibot.setPID(PID_5, 15, 0, 25, 255, -250);

  /* CHECKPOINT SETTINGS */
  /* Parameters = Checkpoint number, Robot will start running from this index */
  /* For example (CP_1, 8), the robot will be ready to execute index 8 */
  ichibot.setCheckPoint (CP_0, 0);
  ichibot.setCheckPoint (CP_1, 1);
  ichibot.setCheckPoint (CP_2, 2);
  ichibot.setCheckPoint (CP_3, 3);
  ichibot.setCheckPoint (CP_4, 4);
  ichibot.setCheckPoint (CP_5, 5);

  /* SET AT WHICH INDEX THE ROBOT WILL STOP */
  ichibot.StopAtIndex (6);

  /* CREATE PATH PLANNING */
  ichibot.setIndex(0, SENSOR_ALL, MOTION,  120 , 120 , 1500 , BLACK_LINE, 101, 250, PID_3,  FAN_OFF, NORMAL);
  ichibot.setIndex(1, SENSOR_LEFT, TURN_LEFT,  100 , BLACK_LINE, 101, 200, PID_3,  FAN_OFF, NORMAL);
  ichibot.setIndex(2, SENSOR_RIGHT, TURN_RIGHT,  100 , BLACK_LINE, 101, 100, PID_3,  FAN_OFF, NORMAL);
  ichibot.setIndex(3, SENSOR_LEFT, TURN_LEFT,  100 , BLACK_LINE, 101, 300, PID_3,  FAN_OFF, NORMAL);
  ichibot.setIndex(4, SENSOR_RIGHT, TURN_RIGHT,  50 , BLACK_LINE, 101, 100, PID_3,  FAN_OFF, NORMAL);
  ichibot.setIndex(5, SENSOR_LEFT_RIGHT, STRAIGHT,  100 , BLACK_LINE, 101, 100, PID_3,  FAN_OFF, NORMAL);
  ichibot.setIndex(6, SENSOR_RIGHT, TURN_LEFT,  100 , BLACK_LINE, 101, 300, PID_5,  FAN_OFF, NORMAL);
  Serial.println("Starting");
}

void loop() {

  ichibot.ichibotLoop();
  
}

// void test_motors(){
  // Serial.println("Start");
  // ichibot.setMotor(120,120);
  // delay(1000);
  // Serial.println("Stop");
  // ichibot.setMotor(0,0);
  // delay(1000);

  // Serial.println("back");
  // ichibot.setMotor(-120,-120);
  // delay(1000);
  // Serial.println("Stop");
  // ichibot.setMotor(0,0);
  // delay(1000);

  // Serial.println("left");
  // ichibot.setMotor(120,0);
  // delay(1000);
  // Serial.println("Stop");
  // ichibot.setMotor(0,0);
  // delay(1000);

  // Serial.println("right");
  // ichibot.setMotor(0,120);
  // delay(1000);
  // Serial.println("Stop");
  // ichibot.setMotor(0,0);
  // delay(1000);
// }