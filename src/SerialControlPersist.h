#ifndef SERIAL_CONTROL_PERSIST_LIB
#define SERIAL_CONTROL_PERSIST_LIB
#include <Arduino.h>
#include <map>
void splitString(const String &input, const String &separator, String &prefix, String &suffix)
{
    int pos = input.indexOf(separator);
    if (pos != -1 && input.substring(pos, pos + separator.length()) == separator)
    {
        prefix = input.substring(0, pos);
        suffix = input.substring(pos + separator.length());
    }
    else
    {
        // If the separator is not found, you can handle it as needed.
        // Here, we set both prefix and suffix to the input string.
        prefix = "";
        suffix = "";
    }
}
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
void SerialControlPersist::setSerial(HardwareSerial& s){
    BluetoothSerial = s;
}
bool SerialControlPersist::update()
{
    if (readSerial())
    {
        String prefix, suffix;
        splitString(incomingMessage, seperator, prefix, suffix);
        if (prefix.length() && suffix.length())
        {
            BluetoothSerial.println("Prefix:<" + prefix + "> Suffix:<" + suffix + ">");
            if (registredINTs.find(prefix) != registredINTs.end())
            {
                int *ptr = registredINTs[prefix];
                *ptr = suffix.toInt();
                return true;
            }

            if (registredFLOATs.find(prefix) != registredFLOATs.end())
            {
                *registredFLOATs[prefix] = suffix.toFloat();
                return true;
            }
            BluetoothSerial.println("failed");
        }

        else
        {
            BluetoothSerial.println("couldn't interpret message");
        }
    }
    return false;
}
bool SerialControlPersist::readSerial()
{

    bool verified = false;
    while (BluetoothSerial.available() > 0)
    {
        char inByte = BluetoothSerial.read();
        if (inByte == ';')
        {
            verified = true;
            break;
        }
        if (inByte != 13 && inByte != 10)
            incomingBuffer = incomingBuffer + inByte;
    }

    if (verified)
    {
        incomingMessage = incomingBuffer;
        incomingBuffer = "";
        BluetoothSerial.println("Message Read: " + incomingMessage );
    }
    return verified;
}

bool SerialControlPersist::isAvailable(String key)
{
    return (registredINTs.find(key) == registredINTs.end()) && (registredFLOATs.find(key) == registredFLOATs.end());
}

void SerialControlPersist::registerINT(String key, int *ptr)
{
    if (isAvailable(key))
    {
        registredINTs[key] = ptr;
    }
}
void SerialControlPersist::registerFLOAT(String key, float *ptr)
{
    if (isAvailable(key))
    {
        registredFLOATs[key] = ptr;
    }
}

SerialControlPersist::SerialControlPersist(/* args */)
{
}
SerialControlPersist::SerialControlPersist(String seperator)
{
    this->seperator = seperator;
}

SerialControlPersist::~SerialControlPersist()
{
}

#endif