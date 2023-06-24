#pragma once
#include <WiFi.h>
#include <ESPmDNS.h>

void onWiFiEvent(WiFiEvent_t event)
{
    switch(event)
    {
        case ARDUINO_EVENT_WIFI_AP_START:
            Serial.println("AP started");
            break;
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            Serial.println("Client connected to AP");
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            Serial.println("Client disconnected from AP");
            break;
        default:
            break;
    }
}
