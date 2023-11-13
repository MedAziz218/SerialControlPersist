
#ifndef IchibotUltimate4s_h
#define IchibotUltimate4s_h
#include <Arduino.h>

// include the library code:
// #include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     4
// Adafruit_SSD1306 lcd(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define PIN_BATT_SENSOR A0
#define PIN_SERVOL 22 //lifter drop   makin kecil makin naik
#define PIN_SERVOR 1 //gripper pick makin kecl makin buka
#define PIN_EXTENGUISHER 4

#define PIN_BTN_UPL 21
#define PIN_BTN_DOWNL 19
#define PIN_BTN_OKL 20

#define PIN_BTN_UPR 2
#define PIN_BTN_DOWNR 11
#define PIN_BTN_OKR 3

#define PIN_FWD_MOTOR_L 14
#define PIN_BWD_MOTOR_L 15

#define PIN_FWD_MOTOR_R 13
#define PIN_BWD_MOTOR_R 12

#define PIN_RX 8
#define PIN_TX 9

#define PIN_EN_SENSOR_L 23
#define PIN_EN_SENSOR_R 0

#define PIN_LED 18

/* DATA DEFINE */
#define NUM_CP 6
#define ACTIVE_LOW 0
#define ACTIVE_HIGH 1

//Servo servo_lift;
//Servo servo_grip;

#define TARUH   0
#define NORMAL  1
#define AMBIL   2

#define PID_0 0
#define PID_1 1
#define PID_2 2
#define PID_3 3
#define PID_4 4
#define PID_5 5

#define CP_0 0
#define CP_1 1
#define CP_2 2
#define CP_3 3
#define CP_4 4
#define CP_5 5


#define FAN_OFF 0
#define FAN_ON  1

#define MOTION ACTION_NOT_USE_SENSOR
#define ACTION ACTION_USE_SENSOR

#define TURN_LEFT            ACTION, -100  , 230
#define TURN_RIGHT           ACTION, 230 , -100
#define STRAIGHT                 ACTION, setting.speed , setting.speed
#define STOP                  ACTION, 0   , 0

#define TWO_CM    0
#define THREE_CM  1
#define MIX       2
#define FOUR_CM   3

#define BLACK_LINE 0
#define WHITE_LINE 1

#define ACTION_NOT_USE_SENSOR 1
#define ACTION_USE_SENSOR 0

#define OR    0
#define EQUAL 1
#define XOR   2

#define SENSOR_ALL          0b11111111111111, 0b00000000000000, OR
#define SENSOR_ALL_HIT     0b11111111111111, 0b00000000000000, EQUAL
#define SENSOR_EMPTY         0b00000000000000, 0b00000000000000, EQUAL
#define SENSOR_LEFT          0b11000000000000, 0b00000000000000, OR
#define SENSOR_RIGHT         0b00000000000011, 0b00000000000000, OR
#define SENSOR_LEFT_RIGHT     0b11100000000000, 0b00000000000111, XOR
#define SENSOR_LEFT_CENTER    0b11100000000000, 0b00000111100000, XOR
#define SENSOR_RIGHT_CENTER   0b00000000000111, 0b00000111100000, XOR
#define SENSOR_ELBOW_RIGHT     0b00000000001110, 0b00000111100000, XOR
#define SENSOR_ELBOW_LEFT      0b01110000000000, 0b00000111100000, XOR


#define A1 1
#define A2 2
struct dataPID {
  byte Kp;
  byte Kd;
  byte Ki;
  byte PMax;
  int PMin;
};

struct dataSetting {
  unsigned int LIMIT_VALUE_SENSOR[14];
  int speed;
  byte thisCheckPoint;
  byte checkPoint[NUM_CP];
  byte stopIndex;
  byte sensor_linewidth;
  byte sensor_sensivity;
  byte lineColor;
  byte numPID;
};

struct dataIndex {
  byte action;
  unsigned int sensorBitValue[2];
  byte modeSensor;
  int L, R, D;
  byte SA;
  unsigned int TA;
  byte lineColor;
  byte fanState, servo_position;
  byte numPID;
};

#define TOTAL_INDEX 99
#define NUM_PID 6
dataIndex ramIndexData[TOTAL_INDEX];
dataPID listPID[NUM_PID];
dataSetting setting;

char buff[100];

static const unsigned char PROGMEM  botLogo[1024] ;
class IchibotUltimate4s {
private:

public:
  void setIndex(byte index, unsigned int sensor0, unsigned int sensor1, byte modeSensor, byte action,  int L , int R , unsigned int D ,
    byte lineColor = BLACK_LINE, byte SA = 150, unsigned int TA  = 0, byte numPID = 0,  byte kipas = 0, byte servo_pos = NORMAL) {
      ramIndexData[index].sensorBitValue[0] = sensor0;
      ramIndexData[index].sensorBitValue[1] = sensor1;
      ramIndexData[index].L = L;
      ramIndexData[index].R = R;
      ramIndexData[index].D = D;
      ramIndexData[index].SA = SA;
      ramIndexData[index].TA = TA;
      ramIndexData[index].modeSensor = modeSensor;
      ramIndexData[index].action = action;
      ramIndexData[index].numPID = numPID;
      ramIndexData[index].fanState = kipas;
      ramIndexData[index].servo_position = servo_pos;
      ramIndexData[index].lineColor = lineColor;
    }

    void StopAtIndex(byte index) {
      setting.stopIndex = index;
      writeSetting();
    }

    void setFan(byte state) {
      digitalWrite(PIN_EXTENGUISHER, state > 0 ? HIGH : LOW);
    }

    void setCheckPoint(byte num, byte index) {
      if (num < NUM_CP) {
        setting.checkPoint[num] = index;
      }
    }

    void setPID(byte num, byte Kp, byte Ki, byte Kd, byte PMax, int PMin) {
      if (num < NUM_PID) {
        listPID[num].Kp = Kp;
        listPID[num].Ki = Ki;
        listPID[num].Kd = Kd;
        listPID[num].PMax = PMax;
        listPID[num].PMin = PMin;
      }
    }

    void setSensorSensivity(byte sensivity) {
      setting.sensor_sensivity = sensivity;
      writeSetting();
    }

    void readSetting() {
      EEPROM.get(0, setting);
    }

    void writeSetting() {
      EEPROM.put(0, setting);
    }

    void begin() {
      Serial.begin(115200);
    //   if (!lcd.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    //     Serial.println(F("SSD1306 allocation failed"));
    //     for (;;);
    //   }

      pinMode(PIN_FWD_MOTOR_L, OUTPUT);
      pinMode(PIN_BWD_MOTOR_L, OUTPUT);
      pinMode(PIN_FWD_MOTOR_R, OUTPUT);
      pinMode(PIN_BWD_MOTOR_R, OUTPUT);
      pinMode(PIN_BTN_OKR, INPUT_PULLUP);
      pinMode(PIN_BTN_OKL, INPUT_PULLUP);
      pinMode(PIN_BTN_DOWNL, INPUT_PULLUP);
      pinMode(PIN_BTN_UPL, INPUT_PULLUP);
      pinMode(PIN_BTN_DOWNR, INPUT_PULLUP);
      pinMode(PIN_BTN_UPR, INPUT_PULLUP);
      pinMode(PIN_LED, OUTPUT);
      pinMode(PIN_EXTENGUISHER, OUTPUT);
      digitalWrite(PIN_EXTENGUISHER, LOW);
      digitalWrite(PIN_LED, HIGH);

    //   lcd.setRotation(2);
    //   lcd.clearDisplay();
    //   lcd.display();
    //   lcd.setTextColor(WHITE);
      setMotor(0, 0);
      readSetting();
      setting.lineColor = ramIndexData[0].lineColor;

      int LOGO_WIDTH = 128;
      int LOGO_HEIGHT = 64;
    //   lcd.drawBitmap(0, 0, botLogo, LOGO_WIDTH, LOGO_HEIGHT, 1);
    //   lcd.display();
      delay(1500);
    }

    void ichibotLoop() {
      while (1) {
        displaySensor();

        // lcd.drawRoundRect(50, 0, 78, 13, 3,  WHITE);

        // lcd.setCursor(57, 3);
        // sprintf(buff,"CP:%.2d(i:%.2d)",setting.thisCheckPoint,setting.checkPoint[setting.thisCheckPoint]);
        // lcd.print(buff);

        // lcd.drawRoundRect(0, 0, 47, 13, 3,  WHITE);
        // lcd.setCursor(3, 3);
        // sprintf(buff, "SPD:%.3d", setting.speed);
        // lcd.print(buff);

        if (digitalRead(PIN_BTN_UPR) == LOW) {
          setting.thisCheckPoint++;
          if (setting.thisCheckPoint > 5) setting.thisCheckPoint = 5;
          delay(150);
          writeSetting();
        }

        if (digitalRead(PIN_BTN_DOWNR) == LOW && setting.thisCheckPoint != 0) {
          setting.thisCheckPoint --;
          if (setting.thisCheckPoint < 0) setting.thisCheckPoint = 0;
          delay(150);
          writeSetting();
        }

        if (digitalRead(PIN_BTN_UPL) == LOW) {
          setting.speed += 5;
          if (setting.speed > 255) setting.speed = 255;
          delay(100);
          writeSetting();
        }

        if (digitalRead(PIN_BTN_DOWNL) == LOW) {
          setting.speed -= 5;
          if (setting.speed < 0) setting.speed = 0;
          delay(100);
          writeSetting();
        }

        // lcd.drawRoundRect(0, 51, 47, 13, 3,  WHITE);
        // lcd.setCursor(5, 51 + 3);
        // lcd.print("OK");
        // lcd.fillTriangle(
        //   19, 51 + 3,
        //   19, 51 + 3 + 6,
        //   19 + 3, 51 + 3 + 3, WHITE);
        //   lcd.setCursor(26, 51 + 3);
        //   lcd.print("Cal");

        //   lcd.drawRoundRect(50, 51, 78, 13, 3,  WHITE);
        //   lcd.setCursor(89, 51 + 3);
        //   lcd.print("CANCEL");
        //   lcd.fillTriangle(
        //     85, 51 + 3,
        //     85, 51 + 3 + 6,
        //     85 - 3, 51 + 3 + 3, WHITE);
        //     lcd.setCursor(62, 51 + 3);
        //     lcd.print("Run");


            if (digitalRead(PIN_BTN_OKL) == LOW) {
              delay(300);
              calibrateSensor();
            }

            // if (digitalRead(PIN_BTN_OKR) == LOW) {
            //   while (digitalRead(PIN_BTN_OKR) == LOW) {
            //     lcd.setCursor(22, 10);
            //     lcd.print("Ready Position");
            //     lcd.setCursor(22, 35);
            //     lcd.print("Release button");
            //     lcd.setCursor(28, 45);
            //     lcd.print("to RUN Robot");

            //     lcd.display();
            //     lcd.clearDisplay();
            //   }
            //   lcd.clearDisplay();
            //   lcd.setTextSize(2);
            //   lcd.setCursor(10, 25);
            //   lcd.print("Robot RUN");
            //   lcd.setTextSize(1);
            //   lcd.display();
            //   break;
            // }

            // lcd.display();
            // lcd.clearDisplay();
          }

          byte thisRunIndex = 0;
          int dataSensor;
          thisRunIndex = setting.checkPoint[setting.thisCheckPoint];
          dataIndex mem  = ramIndexData[thisRunIndex];
          int normalSpeed =  setting.speed;
          setting.lineColor = mem.lineColor;
          int timerSpeed;
          long timer;
            //-------------------------------------------------------------------------important part
          while (1) {
            mem  = ramIndexData[thisRunIndex];
            dataSensor = readSensor();
            byte do_action = 0;
            if ( mem.action == ACTION_NOT_USE_SENSOR) {
              do_action = 1;
            } else if (mem.action == ACTION_USE_SENSOR ) {
              if ( mem.modeSensor == XOR) {
                if (dataSensor & mem.sensorBitValue[0]) {
                  if (dataSensor & mem.sensorBitValue[1])  { // syarat untuk step 1
                    do_action = 2;
                  }
                }
              } else if (mem.modeSensor == OR) {
                if (dataSensor & mem.sensorBitValue[0]) { // syarat untuk step 1
                  do_action = 3;
                }
              } else if (mem.modeSensor == EQUAL) {
                if (dataSensor == mem.sensorBitValue[0]) { // syarat untuk step 1
                  do_action = 4;
                }
              }
            }

            if (do_action) {
              digitalWrite(PIN_LED, LOW);
              setMotor(mem.L, mem.R);
              delay(mem.D);
              setFan(mem.fanState);
              //setServo(mem.servo_position);

              if (mem.action == ACTION_USE_SENSOR ) {
                dataSensor = readSensor();
                while (dataSensor == 0) {
                  dataSensor = readSensor();
                }
              }

              timer = mem.TA;
              timerSpeed = mem.SA;
              setting.numPID = mem.numPID;
              setting.lineColor = mem.lineColor;

              long lastmsg = millis();
              int speedThrottle = timerSpeed / 4;
              while (1) {
                if (speedThrottle < mem.SA) {
                  speedThrottle += 2;
                  setting.speed =  speedThrottle;
                  if (speedThrottle >= mem.SA) setting.speed = timerSpeed;
                }
                dataSensor = readSensor();
                followLine(dataSensor);

                if (digitalRead(PIN_BTN_OKL) == LOW || digitalRead(PIN_BTN_OKR) == LOW) {
                  setMotor(0, 0);
                  break;
                }

                if ((millis()  - lastmsg) > timer) break;
              }
              timer = 0;
              setting.speed = normalSpeed;

              digitalWrite(PIN_LED, HIGH);
              if (thisRunIndex >= setting.stopIndex ) {
                setMotor(0, 0);
                break;
              }
              thisRunIndex ++;
            }

            if (digitalRead(PIN_BTN_OKL) == LOW || digitalRead(PIN_BTN_OKR) == LOW) {
              setMotor(0, 0);
              delay(500);
              break;
            }
            followLine(dataSensor);
          }

        //   while (1) {
        //     lcd.clearDisplay();
        //     lcd.setCursor(32, 10);
        //     lcd.print("Robot Stop");
        //     lcd.setCursor(38, 25);
        //     lcd.print("at Index");
        //     lcd.setTextSize(2);
        //     lcd.setCursor(52, 40);
        //     sprintf(buff, "%.2d", thisRunIndex);
        //     lcd.print(buff);
        //     lcd.setTextSize(1);
        //     digitalWrite(PIN_EN_SENSOR_L, LOW);
        //     digitalWrite(PIN_EN_SENSOR_R, LOW);
        //     setting.lineColor = ramIndexData[0].lineColor;

        //     lcd.display();
        //     if (digitalRead(PIN_BTN_OKR) == LOW) {
        //       thisRunIndex = 0;
        //       delay(300);
        //       break;
        //     }
        //   }
        }

        byte posSensor[14] = {A4, A3, A5, A2, A6, A1, A7, A7, A1, A6, A2, A5, A3, A4};
        int limit_value[14] = {600, 600, 600, 750, 600, 800, 750, 600, 600, 600, 600, 600, 600, 600};
        int adcValue[14];

        int readSensor() {
          unsigned int valSensor[14];
          int dataSensorBit = 0b00000000000000;

          digitalWrite(PIN_EN_SENSOR_L, HIGH);
          digitalWrite(PIN_EN_SENSOR_R, LOW);
          delayMicroseconds(300);
          for (int x = 0; x < 7; x++) {
            adcValue[x] = analogRead(posSensor[x]);
          }
          delayMicroseconds(300);
          digitalWrite(PIN_EN_SENSOR_L, LOW);
          digitalWrite(PIN_EN_SENSOR_R, HIGH);
          delayMicroseconds(300);
          for (int x = 0; x < 7; x++) {
            adcValue[x + 7] = analogRead(posSensor[x + 7]);
          }
          digitalWrite(PIN_EN_SENSOR_L, HIGH);
          digitalWrite(PIN_EN_SENSOR_R, HIGH);

          for (int i = 0; i < 14; i++) {
            if ( adcValue[i] > setting.LIMIT_VALUE_SENSOR[i]) {
              dataSensorBit = dataSensorBit  + (0b10000000000000 >> i);
            }
          }

          int bufBitSensor = 0b111111111111;
          if (setting.lineColor == WHITE_LINE) {
            bufBitSensor = 0b111111111111 - dataSensorBit;
          } else {
            bufBitSensor = dataSensorBit;
          };
          return bufBitSensor;
        }

        void displaySensor() {
          int sens = readSensor();
        //   for (int i = 0; i < 14; i++) {
        //     if ((sens << i) & 0b10000000000000) {
        //       lcd.fillRect(10 + (8 * i), 25, 5, 13, WHITE);
        //     } else {
        //       lcd.fillRect(10 + (8 * i), 25, 5, 13, BLACK);
        //       lcd.fillRect(10 + (8 * i), 25 + 13 - 1, 5, 1, WHITE);
        //     }
        //   }
        }


        double P = 0;
        double D = 0;
        double error = 0;
        double lastError = 0;
        unsigned long lastProcess = 0;

        void followLine (int dataSensor) {
          double deltaTime = (millis() - lastProcess) / 1000.0;
          lastProcess = millis();
          switch (dataSensor) {
            case 0b00000011000000: error = 0;    break;

            case 0b00000110000000: error = 1;    break;
            case 0b00000100000000: error = 2;    break;
            case 0b00001100000000: error = 3;    break;
            case 0b00001000000000: error = 4;    break;
            case 0b00011000000000: error = 5;    break;
            case 0b00010000000000: error = 6;    break;
            case 0b00110000000000: error = 7;    break;
            case 0b00100000000000: error = 8;    break;
            case 0b01100000000000: error = 9;    break;
            case 0b01000000000000: error = 10;    break;
            case 0b11000000000000: error = 12;    break;
            case 0b10000000000000: error = 14;    break;

            case 0b00000001100000: error = -1;    break;
            case 0b00000000100000: error = -2;    break;
            case 0b00000000110000: error = -3;    break;
            case 0b00000000010000: error = -4;    break;
            case 0b00000000011000: error = -5;    break;
            case 0b00000000001000: error = -6;    break;
            case 0b00000000001100: error = -7;    break;
            case 0b00000000000100: error = -8;    break;
            case 0b00000000000110: error = -9;    break;
            case 0b00000000000010: error = -10;    break;
            case 0b00000000000011: error = -12;    break;
            case 0b00000000000001: error = -14;    break;
          }

          P = error * (double) listPID[setting.numPID].Kp;
          D = (error - lastError) * (double) listPID[setting.numPID].Kd / deltaTime;

          double rateError = error - lastError;
          lastError = error;
          int moveVal = (int)P + (int)D;
          int moveLeft = setting.speed - moveVal;
          int moveRight = setting.speed + moveVal;
          if (moveLeft < listPID[setting.numPID].PMin)  moveLeft = listPID[setting.numPID].PMin;
          if (moveLeft > listPID[setting.numPID].PMax)  moveLeft = listPID[setting.numPID].PMax;
          if (moveRight < listPID[setting.numPID].PMin)  moveRight = listPID[setting.numPID].PMin;
          if (moveRight > listPID[setting.numPID].PMax)  moveRight = listPID[setting.numPID].PMax;
          setMotor(moveLeft, moveRight);
        }

        void setMotor(int LL, int RR) {
          if (RR > 0) {
            analogWrite(PIN_FWD_MOTOR_R, 255 - RR);
            analogWrite(PIN_BWD_MOTOR_R, 255);
          } else {
            analogWrite(PIN_FWD_MOTOR_R, 255 );
            analogWrite(PIN_BWD_MOTOR_R, 255 + RR);
          }

          if (LL > 0) {
            analogWrite(PIN_FWD_MOTOR_L, 255 - LL);
            analogWrite(PIN_BWD_MOTOR_L, 255);
          } else {
            analogWrite(PIN_FWD_MOTOR_L, 255);
            analogWrite(PIN_BWD_MOTOR_L, 255 + LL);
          }
        }

        void calibrateSensor() {
          const int numSensor = 14;
          unsigned int minVal[numSensor], maxVal[numSensor];
        //   lcd.clearDisplay();
        //   lcd.setCursor(0, 0);
        //   lcd.print("Calibrate Sensor..");
        //   lcd.setCursor(0, 10);
        //   lcd.print("Move your Robot");
        //   lcd.setCursor(0, 20);
        //   lcd.print("Until touch Line");

        //   lcd.setCursor(0, 40);
        //   lcd.print("Press OK to Finish");
        //   lcd.display();
          for (int i = 0; i < numSensor; i++) {
            minVal[i] = 1023;
            maxVal[i] = 0;
          }
          while (1) {
            int buffSens = readSensor();
            for (int i = 0; i < numSensor; i++) {
              if (adcValue[i] > maxVal[i]) {
                maxVal[i]  = adcValue[i];
              }
              if (adcValue[i] < minVal[i]) {
                minVal[i]  = adcValue[i];
              }
            }
            if (digitalRead(PIN_BTN_OKL) == LOW) {
              delay(300);
              break;
            }
          }

          for (int i = 0; i < numSensor; i++) {
            setting.LIMIT_VALUE_SENSOR[i] = ((maxVal[i] - minVal[i]) * (float)((100.0 - setting.sensor_sensivity) / 100.0)) + minVal[i];
          }
        //   lcd.clearDisplay();
        //   lcd.setCursor(0, 0);
        //   lcd.print("Saving Calibration...");
        //   lcd.display();
          delay(1500);
          writeSetting();
        //   lcd.clearDisplay();
        }

      };
      #endif
