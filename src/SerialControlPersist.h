#ifndef SERIAL_CONTROL_PERSIST_LIB
#define SERIAL_CONTROL_PERSIST_LIB
#include <Arduino.h>
#include <map>
void splitString(const String &input, const String &separator, String &prefix, String &suffix)
{
    int pos = input.indexOf(separator);
    if (pos != -1 && input.substring(pos,pos + separator.length()) == separator)
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
    String seperator = "---";

public:
    SerialControlPersist(/* args */);
    ~SerialControlPersist();
    void registerINT(String key, int *ptr);
    void registerFLOAT(String key, float *ptr);
    bool isAvailable(String key);
    bool readSerial();
    bool update();
   
};
bool SerialControlPersist::update()
{
    if (readSerial())
    {
        String prefix, suffix;
        splitString(incomingMessage, seperator, prefix, suffix);
        if (prefix.length() && suffix.length()){
            // Serial.println("Prefix:<" + prefix + "> Suffix:<" + suffix + ">");
            auto it = registredINTs.find(prefix);
            if (it!=registredINTs.end()){
                int * ptr = it->second ;
                *it->second = suffix.toInt();
            }
            return true;
        }

        else
            Serial.println("couldn't interpret message");
    }
    return false;
}
bool SerialControlPersist::readSerial()
{
    incomingMessage = "";
    while (Serial.available() > 0)
    {
        char inByte = Serial.read();
        incomingMessage = incomingMessage + inByte;
    }
    if (incomingMessage.length())
    {
        Serial.println("Message Read: " + incomingMessage);
    }
    return incomingMessage.length() > 0;
}

bool SerialControlPersist::isAvailable(String key){
    return (registredINTs.find(key) == registredINTs.end()) && (registredFLOATs.find(key) == registredFLOATs.end());
}

void SerialControlPersist::registerINT(String key, int *ptr)
{
    if (isAvailable(key))
    {
        registredINTs[key] = ptr;
    }
    else
    {
        // TODO: add error handling for this case
        //  key already used;
    }
}
void SerialControlPersist::registerFLOAT(String key, float *ptr)
{
    if (isAvailable(key))
    {
        registredFLOATs[key] = ptr;
    }
    else
    {
        // TODO: add error handling for this case
        //  key already used;
    }
}
SerialControlPersist::SerialControlPersist(/* args */)
{
}

SerialControlPersist::~SerialControlPersist()
{
}

#endif