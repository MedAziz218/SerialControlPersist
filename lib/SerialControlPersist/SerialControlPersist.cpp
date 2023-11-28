
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
void SerialControlPersist::setSerial(Stream *s)
{
    Serial.println("changed serial");
    BluetoothSerial = s;
}
void SerialControlPersist::setSeparator(String s)
{
    separator = s;
}
bool SerialControlPersist::update()
{
    if (readSerial())
    {
        String prefix, suffix;
        splitString(incomingMessage, separator, prefix, suffix);
        if (prefix.length() && suffix.length())
        {
            bool success = false;
            // DEBUG
            // BluetoothSerial->println("Prefix:<" + prefix + "> Suffix:<" + suffix + ">");

            if (registredINTs.find(prefix) != registredINTs.end())
            {
                *registredINTs[prefix] = suffix.toInt();
                success = true;
            }

            if (registredFLOATs.find(prefix) != registredFLOATs.end())
            {
                *registredFLOATs[prefix] = suffix.toFloat();
                success = true;
            }

            if (registeredVOIDs_withNoArg.find(prefix) != registeredVOIDs_withNoArg.end())
            {
                // Execute the registered void function with no arguments
                registeredVOIDs_withNoArg[prefix]();
                success = true;
            }

            if (registeredVOIDs_WithStringArg.find(prefix) != registeredVOIDs_WithStringArg.end())
            {
                registeredVOIDs_WithStringArg[prefix](suffix);
                success = true;
            }

            if (success)
            {
                lastKey = prefix;
                lastValue = suffix;
                // DEBUG
                BluetoothSerial->println("success -> " + lastKey + "=" + lastValue);
            }
            else
            {
                BluetoothSerial->println("failed");
            }
            return success;
        }

        else
        {
            // DEBUG
            BluetoothSerial->println("couldn't interpret message");
        }
    }
    lastKey = "";
    lastValue = "";
    return false;
}
bool SerialControlPersist::readSerial()
{
    // BluetoothSerial->println("bex+:"+String(BluetoothSerial->available()));
    bool verified = false;
    while (BluetoothSerial->available() > 0)
    {

        char inByte = BluetoothSerial->read();
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
        // BluetoothSerial->println("Message Read: " + incomingMessage );
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
        registredINTs[key] = ptr;
}
void SerialControlPersist::registerFLOAT(String key, float *ptr)
{
    if (isAvailable(key))
        registredFLOATs[key] = ptr;
}
void SerialControlPersist::registerVoidNoArg(String key, void (*ptr)())
{
    if (isAvailable(key))
        registeredVOIDs_withNoArg[key] = ptr;
}

void SerialControlPersist::registerVoidWithStringArg(String key, void (*ptr)(String))
{
    if (isAvailable(key))
        registeredVOIDs_WithStringArg[key] = ptr;
}

SerialControlPersist::SerialControlPersist()
{
}

SerialControlPersist::SerialControlPersist(Stream *s, String separator)
{
    setSerial(s);
    setSeparator(separator);
}

SerialControlPersist::~SerialControlPersist()
{
}
