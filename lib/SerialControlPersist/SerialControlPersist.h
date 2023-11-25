#ifndef SERIAL_CONTROL_PERSIST_LIB
#define SERIAL_CONTROL_PERSIST_LIB
#include <Arduino.h>
#include <map>

class SerialControlPersist
{
private:
    /* data */
    std::map<String, int *> registredINTs;
    std::map<String, float *> registredFLOATs;
    String incomingMessage = "";
    String incomingBuffer = "";
    String seperator = "-";
    
    
    HardwareSerial& BluetoothSerial = Serial1;
    bool serial_initialized = false;


public:
    String lastKey = "";
    String lastValue ="";
    SerialControlPersist();
    SerialControlPersist(String seperator);
    ~SerialControlPersist();
    void setSerial(HardwareSerial& s);
    void registerINT(String key, int *ptr);
    void registerFLOAT(String key, float *ptr);
    bool isAvailable(String key);
    bool readSerial();
    bool update();
};

#endif