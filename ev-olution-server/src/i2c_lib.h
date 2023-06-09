#include <Arduino.h>
#include "Wire.h"
//#include "PAC194x_5x.h"

#pragma once

#define I2C_PAC_ADDRESS 0x07    // PAC address
#define I2C_SDA         21      // data port
#define I2C_SCL         22      // clock port
#define I2C_CLOCK_RATE  100000  // 100KHz

class I2CLib
{
public:
    bool Init();
    void write(byte *data_in, uint32_t len);
    int  read(byte *data_in, byte *data_out, uint32_t len);

private:

private:
//    Adafruit_INA219 m_ina219;
    bool m_pac_found = false;
    // esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    // i2c_config_t m_config;
};
