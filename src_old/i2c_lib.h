#include <Arduino.h>
#include "Wire.h"
#include <vector>
#include <map>

#pragma once

constexpr int I2C_SDA                       = 21;      // data port
constexpr int I2C_SCL                       = 22;      // clock port
constexpr uint32_t I2C_CLOCK_RATE           = 100000;  // 100KHz
constexpr byte PAC194X5X_REFRESH_ADDR       = 0x00;
constexpr byte PAC194X5X_CTRL_ADDR          = 0x01;
constexpr byte PAC194X5X_VSENSE1_ADDR       = 0x0B;
constexpr byte PAC194X5X_VSENSE2_ADDR       = 0x0C;
constexpr byte PAC194X5X_VBUS1_AVG_ADDR     = 0x0F;
constexpr byte PAC194X5X_VBUS2_AVG_ADDR     = 0x10;
constexpr byte PAC194X5X_VBUS1_ADDR         = 0x07;
constexpr byte PAC194X5X_VBUS2_ADDR         = 0x08;
constexpr byte PAC194X5X_NEG_PWR_FSR_ADDR   = 0x1D;
constexpr byte PAC194X5X_REFRESH_G_ADDR     = 0x1E;
constexpr byte PAC194X5X_REFRESH_V_ADDR     = 0x1F;
constexpr byte PAC194X5X_CTRL_ACT_ADDR      = 0x21;
constexpr byte PAC194X5X_CTRL_LAT_ADDR      = 0x23;

//            channel,        address, bus(1 or 2)
typedef std::map<byte, std::pair<byte, byte>> I2C_MAP;
typedef I2C_MAP::iterator I2C_MAP_ITERATOR;

class I2CLib
{
public:
    bool Init();
    bool write(byte channel, byte *data_in, uint32_t len);
    int  read (byte channel, byte *data_out, uint32_t len);

private:
    I2C_MAP_ITERATOR get_i2c_address_(byte channel);
    bool write_to_channel_(I2C_MAP_ITERATOR i2c_address, byte *data_in, uint32_t data_len);
    int read_(I2C_MAP_ITERATOR i2c_address, byte register_address, byte *data_out, uint32_t data_len);

private:
    bool m_initialized = false;
    I2C_MAP m_i2c_channels;
};
