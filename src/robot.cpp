#include "robot.h"

// Encoders ---------------------------------------------------------
volatile unsigned int encR_ticks = 0;
volatile unsigned int encL_ticks = 0;
void read_rightEncoder() { encR_ticks++; }
void read_leftEncoder() { encL_ticks++; }
unsigned int get_encR() { return encR_ticks; }
unsigned int get_encL() { return encL_ticks; }
void reset_encR() { encR_ticks = 0; }
void reset_encL() { encL_ticks = 0; }
void setupEncoders()
{
    pinMode(PIN_encR_A, INPUT_PULLUP);
    // pinMode(PIN_encR_B, INPUT_PULLUP);
    pinMode(PIN_encL_A, INPUT_PULLUP);
    // pinMode(PIN_encL_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_encR_A), read_rightEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_encL_A), read_leftEncoder, RISING);
}
// ------------------------------------------------------------------

dataPID listPID[NUM_PID];
dataSetting setting;
void Robot::begin()
{
    setupEncoders();
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
    // setSpeed(0);
    setPIDNum(PID_0);
    setLineColor(BLACK_LINE);
}

void Robot::setMotorUntilDelayOrEncoder(int LL, int RR, int DelayMillis, int targetEncL, int targetEncR)
{
    unsigned int startEncL = get_encL();
    unsigned int startEncR = get_encR();
    unsigned long startMillis = millis();

    while ((get_encL() - startEncL < targetEncL) || (get_encR() - startEncR < targetEncR) || (millis() - startMillis < DelayMillis))
    {
        // forwardWithEncoders(startEncL, startEncR, powerL, powerR, lastTimer);
        setMotor(LL, RR);
    };
}
void Robot::forwardUntilSensor(int sensor0, int sensor1, byte modeSensor)
{
    unsigned int startEncL = get_encL();
    unsigned int startEncR = get_encR();
    unsigned long lastTimer = 0;
    int powerL = setting.speed;
    int powerR = setting.speed;
    int dataSensor = readSensor();
    while (!checkForSensorEvent(dataSensor, sensor0, sensor1, modeSensor))
    {
        dataSensor = readSensor();
        forwardWithEncoders(startEncL, startEncR, powerL, powerR, lastTimer);
    };
}
void Robot::forwardUntilDelayOrEncoder(int DelayMillis, int targetEncL, int targetEncR)
{
    unsigned int startEncL = get_encL();
    unsigned int startEncR = get_encR();
    unsigned long startMillis = millis();
    unsigned long lastTimer = 0;
    int powerL = setting.speed;
    int powerR = setting.speed;
    while ((get_encL() - startEncL < targetEncL) || (get_encR() - startEncR < targetEncR) || (millis() - startMillis < DelayMillis))
    {
        forwardWithEncoders(startEncL, startEncR, powerL, powerR, lastTimer);
    };
    // debugSerial->println("forwardUntilDelayOrEncoder started");
    // int i = 0;
    // i++;
    // debugSerial->println(String(i) + "-> EncL:" + String(get_encL() - startEncL) + " EncR:" + String(get_encR() - startEncR));
    // debugSerial->println("forwardUntilDelayOrEncoder ended " + String(i));
}

void Robot::followLineUntilDelayOrEncoder(int DelayMillis, int targetEncL, int targetEncR)
{
    unsigned int startEncL = get_encL();
    unsigned int startEncR = get_encR();
    unsigned long startMillis = millis();
    int dataSensor ;
    while ((get_encL() - startEncL < targetEncL) || (get_encR() - startEncR < targetEncR) || (millis() - startMillis < DelayMillis))
    {
        dataSensor = readSensor();
        followLine(dataSensor);
    }
}

void Robot::followLineUntilSensor(int sensor0, int sensor1, byte modeSensor)
{
    int dataSensor = readSensor();
    while (!checkForSensorEvent(dataSensor, sensor0, sensor1, modeSensor))
    {
        dataSensor = readSensor();
        followLine(dataSensor);
    }
};

void Robot::testPID()
{
    int dataSensor = readSensor();
    followLine(dataSensor);
}
// -----------------------------------

bool Robot::checkForSensorEvent(int dataSensor, int sensor0, int sensor1, byte modeSensor)
{
    bool do_action = false;
    if (modeSensor == XOR)
    {
        if (dataSensor & sensor0)
        {
            if (dataSensor & sensor1)
            {
                do_action = true;
            }
        }
    }
    else if (modeSensor == OR)
    {
        if (dataSensor & sensor0)
        {
            do_action = true;
        }
    }
    else if (modeSensor == EQUAL)
    {
        if (dataSensor == sensor0)
        {
            do_action = true;
        }
    }
    return do_action;
}

bool Robot::checkDelayEvent(int targetMillis, unsigned long startMillis)
{
    if (millis() - startMillis < targetMillis)
    {
        return true; // event still going
    }
    return false; // event done
}
bool Robot::checkEncoderEvent(int encL, int encR, int startEncL, int startEncR)
{
    if ((get_encL() - startEncL < encL) && (get_encR() - startEncR < encR))
    {
        return true;
    }

    return false;
}

// -----------------------------------
void Robot::forwardWithEncoders(unsigned long startEncL, unsigned long startEncR, int &powerL, int &powerR, unsigned long &lastTimer)
{
    const int motor_offset = 1;
    const int max_offset = 10;
    unsigned long diff_l = get_encL() - startEncL;
    unsigned long diff_r = get_encR() - startEncR;
    if (millis() - lastTimer > 0)
    {
        lastTimer = millis();
        if (diff_l > diff_r)
        {
            powerL = powerL - motor_offset;
            powerR = powerR + motor_offset;
        }
        if (diff_l < diff_r)
        {
            powerL = powerL + motor_offset;
            powerR = powerR - motor_offset;
        }
        powerL = constrain(powerL, setting.speed - max_offset, setting.speed + max_offset);
        powerR = constrain(powerR, setting.speed - max_offset, setting.speed + max_offset);
    }
    setMotor(powerL, powerR);
}
void Robot::followLine(int dataSensor)
{
    double deltaTime = (micros() - lastProcess) / 1000000.0;
    lastProcess = micros();
    // clang-format off
    switch (dataSensor) {
        case 0b00011000: error = 0;    break;

        case 0b00110000: error = 1;    break;
        case 0b00100000: error = 2;    break;
        case 0b01100000: error = 3;    break;
        case 0b01000000: error = 4;    break;
        case 0b11000000: error = 5;    break;
        case 0b10000000: error = 6;    break;

        case 0b00001100: error = -1;   break;
        case 0b00000100: error = -2;   break;
        case 0b00000110: error = -3;   break;
        case 0b00000010: error = -4;   break;
        case 0b00000011: error = -5;   break;
        case 0b00000001: error = -6;   break;
    }
    // clang-format on
    // debugSerial->println("--> error: " + String(error));
    // displaySensor(dataSensor);

    // 0b101100; 0b100100; 0b101111
    // if ((dataSensor & 0b100000) && 
    //     (dataSensor & 0b001111))
    if ((dataSensor &0b10000000) && 
        (dataSensor &0b00011111))
    {
        error = lastOnLineError;
    }
    else if (dataSensor != 0b00000000)
    {
        lastOnLineError = error;
    }
    else if (dataSensor == 0b00000000){
        error = lastOnLineError;
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
    setMotor(moveLeft, moveRight);
    // DEBUG
    if (this->DEBUG_Pid)
    {
        displaySensor(dataSensor);
        debugSerial->println("num_pid: " + String(setting.numPID) + " kp: " + String(listPID[setting.numPID].Kp) + " kd: " + String(listPID[setting.numPID].Kd));

        debugSerial->println("error: " + String(error, 5) + " max: " + String(listPID[setting.numPID].PMax) + " min: " + String(listPID[setting.numPID].PMin));
        debugSerial->println("moveVal:" + String(moveVal) + " mv:" + String(mv) + " deltaTime:" + String(deltaTime, 5));
        debugSerial->println("moveLeft:" + String(moveLeft) + " moveRight:" + String(moveRight));
        debugSerial->println("speed:" + String(setting.speed) + " lastError:" + String(lastError, 5));
        debugSerial->println("------------------------");
        this->DEBUG_Pid = 0;
    }
}

void Robot::setMotor(int LL, int RR)
{
    // LL = constrain(LL, -255, 255);
    // RR = constrain(RR, -255, 255);
    if (!ON)
    {
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
int Robot::readSensor()
{
    int dataSensorBit = 0b00000000;

    for (int x = 0; x < sensorCount; x++)
    {
        if (digitalRead(posSensor[x]))
        {
            dataSensorBit = dataSensorBit + (0b10000000 >> x);
        };
    }
    int bufBitSensor = 0b11111111;
    if (setting.lineColor == WHITE_LINE)
    {
        bufBitSensor = 0b11111111 - dataSensorBit;
    }
    else
    {
        bufBitSensor = dataSensorBit;
    };
    return bufBitSensor;
}

void Robot::displaySensor(int sens)
{
    String s = "+00000000+";
    for (int i = 0; i < sensorCount; i++)
    {
        s[i + 1] = sens & (0b10000000 >> i) ? '1' : '0';
    }
    debugSerial->println(s);
}

void Robot::setPID(byte num, byte Kp, byte Ki, byte Kd, byte PMax, int PMin)
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

void Robot::debugCode()
{
    if (DEBUG_Encoders)
    {
        displaySensor(readSensor());
        Serial1.println("encL:" + String(get_encL()) + " encR:" + String(get_encR())); // DEBUG
        DEBUG_Encoders = 0;
    }
    if (DEBUG_Pid)
    {
        testPID();
        DEBUG_Pid = 0;
    }
    if (DEBUG_ResetEnc)
    {
        reset_encL();
        reset_encR();
        DEBUG_ResetEnc = 0;
    }
}