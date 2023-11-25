
#include "SerialControlPersist.h"

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
            bool success=false;
            // DEBUG
            // BluetoothSerial.println("Prefix:<" + prefix + "> Suffix:<" + suffix + ">");
            if (registredINTs.find(prefix) != registredINTs.end())
            {
                // int *ptr = registredINTs[prefix];
                // *ptr = suffix.toInt();
                *registredINTs[prefix] = suffix.toInt();
                lastKey = prefix;
                lastValue = suffix ;
                success = true;
            }

            if (registredFLOATs.find(prefix) != registredFLOATs.end())
            {
                *registredFLOATs[prefix] = suffix.toFloat();
                lastKey = prefix;
                lastValue = suffix ;
                success = true;
            }

            // DEBUG
            if (success){
                BluetoothSerial.println("success -> "+lastKey+"="+lastValue);
            }else {
                BluetoothSerial.println("failed");
            }
            return success;
        }

        else
        {   
            // DEBUG
            BluetoothSerial.println("couldn't interpret message");
        }
    }
    lastKey = "";
    lastValue = "";
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
        // DEBUG
        // BluetoothSerial.println("Message Read: " + incomingMessage );
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

SerialControlPersist::SerialControlPersist()
{
}
SerialControlPersist::SerialControlPersist(String seperator)
{
    this->seperator = seperator;
}

SerialControlPersist::~SerialControlPersist()
{
}
