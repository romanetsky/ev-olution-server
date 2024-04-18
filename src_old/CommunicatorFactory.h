#pragma once
#include "SerialCommunicator.h"
#include "WiFiCommunicator.h"

#define DEF_COMMUNICATOR_SERIAL  1
#define DEF_COMMUNICATOR_WIFI    2

class CommunicatorFactory
{
public:
    static BaseCommunicator* createCommunicator(int type)
    {
        BaseCommunicator* comm = nullptr;
        switch (type)
        {
        case DEF_COMMUNICATOR_SERIAL:
            comm = new SerialCommunicator();
            break;
        case DEF_COMMUNICATOR_WIFI:
            comm = new WiFiCommunicator(); 
            break;
        default:
            break;
        }

        return comm;
    }
};
