#ifndef ROBOT_H
#define ROBOT_H
#include <Arduino.h>
#include <SerialControlPersist.h>

#define PWM_Res 8
#define PWM_Freq 1200

#define PWM_FWD_MOTOR_L 0
#define PWM_BWD_MOTOR_L 1
#define PWM_FWD_MOTOR_R 2
#define PWM_BWD_MOTOR_R 3

#define PIN_FWD_MOTOR_R 26
#define PIN_BWD_MOTOR_R 25
#define PIN_encR_A 35

#define PIN_FWD_MOTOR_L 32
#define PIN_BWD_MOTOR_L 33
#define PIN_encL_A 39

#define NUM_PID 6
#define PID_0 0
#define PID_1 1
#define PID_2 2
#define PID_3 3
#define PID_4 4
#define PID_5 5

#define OR 0
#define EQUAL 1
#define XOR 2

#define BLACK_LINE 0
#define WHITE_LINE 1

#define SENSOR_ALL 0b111111, 0b000000, OR
#define SENSOR_ALL_HIT 0b111111, 0b000000, EQUAL
#define SENSOR_EMPTY 0b000000, 0b000000, EQUAL
#define SENSOR_LEFT 0b100000, 0b000000, OR
#define SENSOR_RIGHT 0b000001, 0b000000, OR
#define SENSOR_LEFT_RIGHT 0b100000, 0b000001, XOR
#define SENSOR_LEFT_CENTER 0b100000, 0b001100, XOR
#define SENSOR_RIGHT_CENTER 0b000001, 0b001100, XOR
#define SENSOR_ELBOW_RIGHT 0b000010, 0b001100, XOR
#define SENSOR_ELBOW_LEFT 0b010000, 0b001000, XOR

#define sensorCount 6
struct dataPID
{
    byte Kp;
    byte Kd;
    byte Ki;
    byte PMax;
    int PMin;
};

struct dataSetting
{
    int speed;
    int stopIndex;
    byte lineColor;
    byte numPID;
};

struct dataIndex
{
    byte action;
    unsigned int sensorBitValue[2];
    byte modeSensor;
    int L, R;
    int D;
    int encL, encR;
    byte SA;
    unsigned int TA;
    unsigned int encL_A;
    unsigned int encR_A;
    byte lineColor;
    byte numPID;
};
extern dataPID listPID[NUM_PID];
extern dataSetting setting;
class Robot
{
private:
    double P = 0;
    double D = 0;
    double error = 0;
    double lastError = 0;
    double lastOnLineError = 0;
    unsigned long lastProcess = 0;
    bool checkForSensorEvent(int dataSensor, int sensor0, int sensor1, byte modeSensor);
    bool checkEncoderEvent(int encL, int encR, int startEncL, int startEncR);
    bool checkDelayEvent(int targetMillis, unsigned long startMillis);
    Stream *debugSerial;
    byte posSensor[sensorCount] = {23, 22, 21, 19, 18, 5};
    void followLine(int dataSensor);
    void forwardWithEncoders(unsigned long startEncL, unsigned long startEncR, int &powerL, int &powerR, unsigned long &lastTimer);
    
public:

    void begin();
    void debugLoop();
    int readSensor();
    void displaySensor(int sens);
    void setMotor(int LL, int RR);
    void setPID(byte num, byte Kp, byte Ki, byte Kd, byte PMax, int PMin);
    void testPID();

    void followLineUntilDelayOrEncoder(int DelayMillis, int targetEncL, int targetEncR);
    void forwardUntilDelayOrEncoder(int DelayMillis, int targetEncL, int targetEncR);
    void followLineUntilSensor(int sensor0, int sensor1, byte modeSensor);
    void forwardUntilSensor(int sensor0, int sensor1, byte modeSensor);
    void setMotorUntilDelayOrEncoder(int LL, int RR, int DelayMillis, int targetEncL, int targetEncR );
    
    inline void setSpeed(int speed) { setting.speed = speed; }
    inline void StopAtIndex(int stopIndex) { setting.stopIndex = stopIndex; }
    inline void setLineColor(byte lineColor) { setting.lineColor = lineColor; }
    inline void setPIDNum(byte numPID) { setting.numPID = numPID; }
    inline void setDebugSerial(Stream *debugSerial) { this->debugSerial = debugSerial; }
    inline void setState(int state) { this->ON = state; };

    int ON = 0;
    int DEBUG_Pid = 0;
    int DEBUG_Encoders = 0;
    int DEBUG_ResetEnc = 0;

    // void turnRight();
    // void turnLeft();
    // void spinRight();
    // void spinLeft();
};
#endif