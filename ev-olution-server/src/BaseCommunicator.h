#pragma once

class BaseCommunicator
{
public:
    virtual int available() = 0;
    virtual int read(byte* buffer, int size_bytes) = 0;
    virtual int write(byte* buffer, int size_bytes) = 0;
};
