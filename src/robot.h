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
// clang-format off
#define SENSOR_ALL          0b11111111, 0b00000000, OR
#define SENSOR_ALL_HIT      0b11111111, 0b00000000, EQUAL
#define SENSOR_EMPTY        0b00000000, 0b00000000, EQUAL
#define SENSOR_LEFT         0b11000000, 0b00000000, OR
#define SENSOR_RIGHT        0b00000011, 0b00000000, OR
#define SENSOR_LEFT_RIGHT   0b11000000, 0b00000011, XOR
#define SENSOR_LEFT_CENTER  0b11000000, 0b00011000, XOR
#define SENSOR_RIGHT_CENTER 0b00000011, 0b00011000, XOR
#define SENSOR_ELBOW_RIGHT  0b00000110, 0b00011000, XOR
#define SENSOR_ELBOW_LEFT   0b01100000, 0b00011000, XOR
#define SENSOR_WIDE_CENTER  0b00100100, 0b00011000, XOR
// clang-format on
#define sensorCount 8
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
extern unsigned int get_encL();
extern unsigned int get_encR();

class Robot
{
private:
    int shiftCoeff = 0;
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
    byte posSensor[sensorCount] = {15,23, 22, 21, 19, 18, 5,14};
    void followLine(int dataSensor);
    void forwardWithEncoders(unsigned long startEncL, unsigned long startEncR, int &powerL, int &powerR, unsigned long &lastTimer);

public:
    void begin();
    void debugCode();
    int readSensor();
    void displaySensor(int sens);
    void setMotor(int LL, int RR);
    void setPID(byte num, byte Kp, byte Ki, byte Kd, byte PMax, int PMin);
    void testPID();

    void followLineUntilDelayOrEncoder(int DelayMillis, int targetEncL, int targetEncR);
    void forwardUntilDelayOrEncoder(int DelayMillis, int targetEncL, int targetEncR);
    void followLineUntilSensor(int sensor0, int sensor1, byte modeSensor);
    void forwardUntilSensor(int sensor0, int sensor1, byte modeSensor);
    void setMotorUntilDelayOrEncoder(int LL, int RR, int DelayMillis, int targetEncL, int targetEncR);

    inline void setSpeed(int speed) { setting.speed = speed; }
    inline void StopAtIndex(int stopIndex) { setting.stopIndex = stopIndex; }
    inline void setLineColor(byte lineColor) { setting.lineColor = lineColor; }
    inline void setPIDNum(byte numPID) { setting.numPID = numPID; }
    inline void setDebugSerial(Stream *debugSerial) { this->debugSerial = debugSerial; }
    inline void setState(int state) { this->ON = state; };
    
    /**
     * Sets the shift coefficient for centering.
     *
     * @param coeff +2 ligne ywalli 3al isar , -2 3al imin
     */
    void shiftCenter(int coeff){
        shiftCoeff = coeff;
    }
    void STOP (){
        setState(0);
        setMotor(0,0);
    }

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