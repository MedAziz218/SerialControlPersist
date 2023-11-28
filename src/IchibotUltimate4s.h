
#ifndef IchibotUltimate4s_h
#define IchibotUltimate4s_h
#include <Arduino.h>

// include the library code:
// #include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#define PWM_Res 8
#define PWM_Freq 1200

#define PWM_FWD_MOTOR_L 0
#define PWM_BWD_MOTOR_L 1
#define PWM_FWD_MOTOR_R 2
#define PWM_BWD_MOTOR_R 3

#define PIN_FWD_MOTOR_R 26
#define PIN_BWD_MOTOR_R 25

#define PIN_FWD_MOTOR_L 32
#define PIN_BWD_MOTOR_L 33

#define PIN_RX 8
#define PIN_TX 9

/* DATA DEFINE */
#define NUM_CP 6
#define ACTIVE_LOW 0
#define ACTIVE_HIGH 1

// Servo servo_lift;
// Servo servo_grip;

#define TARUH 0
#define NORMAL 1
#define AMBIL 2

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
#define FAN_ON 1

#define MOTION ACTION_NOT_USE_SENSOR
#define ACTION ACTION_USE_SENSOR

#define TURN_LEFT ACTION, -100, 230
#define TURN_RIGHT ACTION, 230, -100
#define STRAIGHT ACTION, setting.speed, setting.speed
#define STOP ACTION, 0, 0

#define BLACK_LINE 0
#define WHITE_LINE 1

#define ACTION_NOT_USE_SENSOR 1
#define ACTION_USE_SENSOR 0

#define OR 0
#define EQUAL 1
#define XOR 2

#define SENSOR_ALL 0b111111, 0b000000, OR
#define SENSOR_ALL_HIT 0b111111, 0b000000, EQUAL
#define SENSOR_EMPTY 0b000000, 0b000000, EQUAL
#define SENSOR_LEFT 0b100000, 0b000000, OR
#define SENSOR_RIGHT 0b000001, 0b000000, OR
#define SENSOR_LEFT_RIGHT 0b100000, 0b000001, XOR
#define SENSOR_LEFT_CENTER 0b100000, 0b001100, XOR
#define SENSOR_RIGHT_CENTER 0b000001, 0b001100, XOR
#define SENSOR_ELBOW_RIGHT 0b000010, 0b001100, XOR
#define SENSOR_ELBOW_LEFT 0b010000, 0b001100, XOR

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
    unsigned int LIMIT_VALUE_SENSOR[sensorCount];
    int speed;
    byte thisCheckPoint;
    byte checkPoint[NUM_CP];
    byte stopIndex;
    byte sensor_linewidth;
    byte sensor_sensivity;
    byte lineColor;
    byte numPID;
};

struct dataIndex
{
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

// char buff[100];
// static const unsigned char PROGMEM botLogo[1024];
class IchibotUltimate4s
{
private:
public:
    void setIndex(byte index, unsigned int sensor0, unsigned int sensor1, byte modeSensor, byte action, int L, int R, unsigned int D,
                  byte lineColor = BLACK_LINE, byte SA = 150, unsigned int TA = 0, byte numPID = 0, byte FanState = 0, byte servo_pos = NORMAL)
    {
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
        ramIndexData[index].fanState = FanState;
        ramIndexData[index].servo_position = servo_pos;
        ramIndexData[index].lineColor = lineColor;
    }

    void StopAtIndex(byte index)
    {
        setting.stopIndex = index;
        writeSetting();
    }

    void setCheckPoint(byte num, byte index)
    {
        if (num < NUM_CP)
        {
            setting.checkPoint[num] = index;
        }
    }

    void setPID(byte num, byte Kp, byte Ki, byte Kd, byte PMax, int PMin)
    {
        if (num < NUM_PID)
        {
            listPID[num].Kp = Kp;
            listPID[num].Ki = Ki;
            listPID[num].Kd = Kd;
            listPID[num].PMax = PMax;
            listPID[num].PMin = PMin;
        }
    }
    void setFan(byte state)
    {
    }

    void setSensorSensivity(byte sensivity)
    {
        setting.sensor_sensivity = sensivity;
        writeSetting();
    }

    void readSetting()
    {
        EEPROM.get(0, setting);
    }

    void writeSetting()
    {
        EEPROM.put(0, setting);
    }

    void begin()
    {
        Serial.begin(115200);

        // pinMode(hbridgeA,OUTPUT); pinMode(hbridgeB,OUTPUT);
        // pinMode(encoderPinA,INPUT);pinMode(encoderPinB,INPUT);
        pinMode(PIN_FWD_MOTOR_L, OUTPUT);
        ledcSetup(PWM_FWD_MOTOR_L, PWM_Freq, PWM_Res);
        ledcAttachPin(PIN_FWD_MOTOR_L, PWM_FWD_MOTOR_L);

        pinMode(PIN_BWD_MOTOR_L, OUTPUT);
        ledcSetup(PWM_BWD_MOTOR_L, PWM_Freq, PWM_Res);
        ledcAttachPin(PIN_BWD_MOTOR_L, PWM_BWD_MOTOR_L);

        pinMode(PIN_FWD_MOTOR_R, OUTPUT);
        ledcSetup(PWM_FWD_MOTOR_R, PWM_Freq, PWM_Res);
        ledcAttachPin(PIN_FWD_MOTOR_R, PWM_FWD_MOTOR_R);

        pinMode(PIN_BWD_MOTOR_R, OUTPUT);
        ledcSetup(PWM_BWD_MOTOR_R, PWM_Freq, PWM_Res);
        ledcAttachPin(PIN_BWD_MOTOR_R, PWM_BWD_MOTOR_R);

        for (int i = 0; i < sensorCount; i++)
            pinMode(posSensor[i], INPUT);

        setMotor(0, 0);
        readSetting();
        setting.lineColor = ramIndexData[0].lineColor;
        setting.speed = 0;
        delay(1500);
    }
    SimpleTimer displaySensorTimer;
    void ichibotLoop()
    {

        byte thisRunIndex = 0;
        int dataSensor;
        thisRunIndex = setting.checkPoint[setting.thisCheckPoint];
        dataIndex mem = ramIndexData[thisRunIndex];
        int normalSpeed = setting.speed;
        setting.lineColor = mem.lineColor;
        int timerSpeed;
        long timer;
        //-------------------------------------------------------------------------important part
        while (1)
        {
            mem = ramIndexData[thisRunIndex];
            dataSensor = readSensor();
            // DEBUG
            if (displaySensorTimer.isReady())
            {
                displaySensor(dataSensor);
                displaySensorTimer.setInterval(500);
                displaySensorTimer.reset();
            }
            
            byte do_action = 0;
            if (mem.action == ACTION_NOT_USE_SENSOR)
            {
                do_action = 1;
            }
            else if (mem.action == ACTION_USE_SENSOR)
            {
                if (mem.modeSensor == XOR)
                {
                    if (dataSensor & mem.sensorBitValue[0])
                    {
                        if (dataSensor & mem.sensorBitValue[1])
                        {
                            do_action = 2;
                        }
                    }
                }
                else if (mem.modeSensor == OR)
                {
                    if (dataSensor & mem.sensorBitValue[0])
                    {
                        do_action = 3;
                    }
                }
                else if (mem.modeSensor == EQUAL)
                {
                    if (dataSensor == mem.sensorBitValue[0])
                    {
                        do_action = 4;
                    }
                }
            }

            if (do_action)
            {

                Serial.println("doing action");
                // digitalWrite(PIN_LED, LOW);
                setMotor(mem.L, mem.R);
                delay(mem.D);
                // setFan(mem.fanState);
                // setServo(mem.servo_position);

                if (mem.action == ACTION_USE_SENSOR)
                {
                    dataSensor = readSensor();
                    if (dataSensor == 0)
                        Serial.println("waiting for line");
                    while (dataSensor == 0)
                    {
                        dataSensor = readSensor();
                    }
                }

                timer = mem.TA;
                timerSpeed = mem.SA;
                setting.numPID = mem.numPID;
                setting.lineColor = mem.lineColor;

                long lastmsg = millis();
                int speedThrottle = timerSpeed / 4;
                while (1)
                {
                    if (speedThrottle < mem.SA)
                    {
                        speedThrottle += 2;
                        setting.speed = speedThrottle;
                        if (speedThrottle >= mem.SA)
                            setting.speed = timerSpeed;
                    }
                    dataSensor = readSensor();
                    followLine(dataSensor);

                    if ((millis() - lastmsg) > timer)
                    {
                        // DEBUG
                        Serial.println("Finished Action at index: " + String(thisRunIndex));
                        break;
                    }
                }
                timer = 0;
                setting.speed = normalSpeed;

                if (thisRunIndex >= setting.stopIndex)
                {
                    // DEBUG
                    Serial.println("Stopping at index: " + String(thisRunIndex));

                    setMotor(0, 0);
                    ON = 0;
                    break;
                }
                thisRunIndex++;
                // DEBUG
                Serial.println("incrementing index to : " + String(thisRunIndex));
            }
            // DEBUG
            // Serial.println("following line at index: "+String(thisRunIndex));
            followLine(dataSensor);
        }
    }

    byte posSensor[sensorCount] = {23, 22, 21, 19, 18, 5};
    int limit_value[sensorCount]; // = {600, 600, 600, 750, 600, 800, 750, 600, 600, 600, 600, 600, 600, 600};
    int adcValue[sensorCount];

    int readSensor()
    {
        unsigned int valSensor[6];
        int dataSensorBit = 0b000000;

        // digitalWrite(PIN_EN_SENSOR_L, HIGH);
        // digitalWrite(PIN_EN_SENSOR_R, LOW);

        for (int x = 0; x < sensorCount; x++)
        {
            adcValue[x] = digitalRead(posSensor[x]);
        }
        delayMicroseconds(300);
        for (int i = 0; i < sensorCount; i++)
        {
            if (adcValue[i] > 0)
            {
                dataSensorBit = dataSensorBit + (0b100000 >> i);
            }
        }
        int bufBitSensor = 0b111111;
        if (setting.lineColor == WHITE_LINE)
        {
            bufBitSensor = 0b111111 - dataSensorBit;
        }
        else
        {
            bufBitSensor = dataSensorBit;
        };
        return bufBitSensor;
    }

    void displaySensor(int sens)
    {
        String s="+000000+";
        for (int i = 0; i < sensorCount; i++)
        {
            s[i+1] = sens & (0b100000 >> i) ? '1' : '0';
        }
       
        Serial1.println(s);
    }

    double P = 0;
    double D = 0;
    double error = 0;
    double lastError = 0;
    unsigned long lastProcess = 0;
    // DEBUG
    int DEBUG_LOG = 0;
    int ON = 0;
    void followLine(int dataSensor)
    {
        double deltaTime = (micros() - lastProcess) / 1000000.0;
        lastProcess = micros();
        switch (dataSensor)
        {
        //+001110+
        case 0b001100:
            error = 0;
            break;
        case 0b011110:
            error = 0;
            break;
        case 0b111111:
            error = 0;
            break;

        case 0b011000:
            error = 1;
            break;
        case 0b010000:
            error = 2;
            break;
        case 0b110000:
            error = 3;
            break;
        case 0b100000:
            error = 4;
            break;

        case 0b000110:
            error = -1;
            break;
        case 0b000010:
            error = -2;
            break;
        case 0b000011:
            error = -3;
            break;
        case 0b000001:
            error = -4;
            break;
        }
        P = error * (double)listPID[setting.numPID].Kp;
        D = (error - lastError) * (double)listPID[setting.numPID].Kd / deltaTime;

        double rateError = error - lastError;
        lastError = error;

        double mv = P + D;
        int moveVal = (int)(P + D);
        int moveLeft = setting.speed - moveVal;
        int moveRight = setting.speed + moveVal;

        if (moveLeft < listPID[setting.numPID].PMin)
            moveLeft = listPID[setting.numPID].PMin;
        if (moveLeft > listPID[setting.numPID].PMax)
            moveLeft = listPID[setting.numPID].PMax;
        if (moveRight < listPID[setting.numPID].PMin)
            moveRight = listPID[setting.numPID].PMin;
        if (moveRight > listPID[setting.numPID].PMax)
            moveRight = listPID[setting.numPID].PMax;
        // DEBUG
        if (DEBUG_LOG)
        {
            displaySensor(dataSensor);
            Serial1.println("error: "+String(error,5));
            Serial1.println("moveVal:" + String(moveVal) + " mv:" + String(mv) + " deltaTime:" + String(deltaTime, 5));
            Serial1.println("moveLeft:" + String(moveLeft) + " moveRight:" + String(moveRight));
            Serial1.println("speed:" + String(setting.speed) + " lastError:" + String(lastError, 5));
            Serial1.println("------------------------");
            DEBUG_LOG = 0;
        }
        
        setMotor(moveLeft, moveRight);
        
    }

    void setMotor(int LL, int RR)
    {
        if (!ON){
            ledcWrite(PWM_FWD_MOTOR_R, 0);
            ledcWrite(PWM_BWD_MOTOR_R, 0);
            ledcWrite(PWM_FWD_MOTOR_L, 0);
            ledcWrite(PWM_BWD_MOTOR_L, 0);
            return;
        }
        if (RR > 0)
        {
            ledcWrite(PWM_FWD_MOTOR_R, RR);
            ledcWrite(PWM_BWD_MOTOR_R, 0);
        }
        else
        {
            ledcWrite(PWM_FWD_MOTOR_R, 0);
            ledcWrite(PWM_BWD_MOTOR_R, -RR);
        }

        if (LL > 0)
        {
            ledcWrite(PWM_FWD_MOTOR_L, LL);
            ledcWrite(PWM_BWD_MOTOR_L, 0);
        }
        else
        {
            ledcWrite(PWM_FWD_MOTOR_L, 0);
            ledcWrite(PWM_BWD_MOTOR_L, -LL);
        }
    }

    void testPID()
    {
        int dataSensor = readSensor();
        followLine(dataSensor);
    }
};
#endif
