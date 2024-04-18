#pragma once
#include <Arduino.h>
#include <ESPmDNS.h>
#include "wifi_lib.h"
#include "BaseCommunicator.h"

const char* wifi_ssid = "EVONET";
const char* wifi_password = "12345678";
const char* wifi_hostname = "my_evo_host";
const uint16_t wifi_port = 4556;
WiFiServer wifi_server(wifi_port);

class WiFiCommunicator : public BaseCommunicator
{
public:
    WiFiCommunicator()
    {
        // wifi setup
        WiFi.onEvent(onWiFiEvent);
        WiFi.mode(WIFI_AP);
        WiFi.softAP(wifi_ssid, wifi_password);

        // multicast DNS
        MDNS.begin(wifi_hostname);
        MDNS.addService("http", "tcp", 80);
    
        WiFi.setHostname(wifi_hostname);
        wifi_server.begin();

//         int nrOfServices = MDNS.queryService("http", "tcp");
// Serial.print("Number of services found: ");
// Serial.println(nrOfServices);
// for (int i = 0; i < nrOfServices; i=i+1) {
//     Serial.println("---------------");
//     Serial.print("Hostname: ");
//     Serial.println(MDNS.hostname(i));
//     Serial.print("IP address: ");
//     Serial.println(MDNS.IP(i));
//     Serial.print("Port: ");
//     Serial.println(MDNS.port(i));
//     Serial.println("---------------");
// }
    }
    virtual int available() override
    {
        if (!client)
        {
            client = wifi_server.available();
        }
        if (client)
        {
            return (int)client.available();
        }
        else
        {
            return 0;
        }
    }
    virtual int read(byte *buffer, int size_bytes) override
    {
        if (!client)
            return 0;

        return (int)client.readBytes(buffer, size_bytes);
    }
    virtual int write(byte *buffer, int size_bytes) override
    {
        if (!client)
            return 0;
        
        int write_size = (int)client.write(buffer, size_bytes);
        client.flush();
        
        return write_size;
    }

private:
    WiFiClient client;
};
