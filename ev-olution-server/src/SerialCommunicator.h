#pragma once
#include <Arduino.h>
#include "BaseCommunicator.h"

#define SerialBaudRate 115200
#define SerialTimeOutMS 1

class SerialCommunicator : public BaseCommunicator
{
public:
    SerialCommunicator()
    {
        Serial.begin(SerialBaudRate);
        Serial.setTimeout(SerialTimeOutMS);
    }
    virtual int available() override
    {
        return (int)Serial.available();
    }
    virtual int read(byte *buffer, int size_bytes) override
    {
        return (int)Serial.readBytes(buffer, size_bytes);
    }
    virtual int write(byte *buffer, int size_bytes) override
    {
        int write_size = (int)Serial.write(buffer, size_bytes);
        Serial.flush();

        return write_size;
    }
};
