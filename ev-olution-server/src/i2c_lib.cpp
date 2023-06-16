#include "i2c_lib.h"

bool I2CLib::Init()
{
    m_initialized = true;

    Wire.begin(I2C_SDA, I2C_SCL, I2C_CLOCK_RATE);

    // scan for I2C devices,
    // and build mapping table translating channel into touple <pac_addr,bus>
    byte error, i2c_address;
    byte channel = 0;
    for (i2c_address = 1; i2c_address < 127; i2c_address++)
    {
        Wire.beginTransmission(i2c_address);
        delay(1);
        error = Wire.endTransmission();
        if (error == 0)
        {
// Serial.print("found ");
// Serial.print(channel);
// Serial.print(" channel at address 0x");
// Serial.println(i2c_address, HEX);
            m_i2c_channels[channel++] = std::pair<byte,byte>(i2c_address, 1);
            m_i2c_channels[channel++] = std::pair<byte,byte>(i2c_address, 2);
        }
    }

    byte data[3];
    for (I2C_MAP_ITERATOR i2c_iter = m_i2c_channels.begin(); i2c_iter != m_i2c_channels.end(); i2c_iter++)
    {
        data[0] = PAC194X5X_CTRL_ADDR;
        data[1] = 0x00;
        data[2] = 0x00;
        m_initialized |= write_to_channel_(i2c_iter, data, 3);
delay(3);
        data[0] = PAC194X5X_NEG_PWR_FSR_ADDR;
        data[1] = 0x55;
        data[2] = 0x55;
        m_initialized |= write_to_channel_(i2c_iter, data, 3);
delay(3);
        data[0] = PAC194X5X_REFRESH_ADDR;
        m_initialized |= write_to_channel_(i2c_iter, data, 1);
delay(3);
    }

// byte data1[4];
// for (byte channel = 0; channel < 16; channel++)
// {
// data1[0] = 9;
// data1[1] = 9;
// data1[2] = 9;
// data1[3] = 9;
// int a = read(channel, data1, 2);
// Serial.print("channel ");
// Serial.print(channel);
// Serial.print(": measured values (");
// Serial.print(a);
// Serial.print("): ");
// Serial.print(data1[0]); Serial.print(", ");
// Serial.print(data1[1]); Serial.print(", ");
// Serial.print(data1[2]); Serial.print(", ");
// Serial.print(data1[3]);
// Serial.println("...");
// delay(100);
// }
    return m_initialized;
}

bool I2CLib::write(byte channel, byte *data_in, uint32_t len)
{
    return write_to_channel_(get_i2c_address_(channel), data_in, len);
}

bool I2CLib::write_to_channel_(I2C_MAP_ITERATOR i2c_iter, byte *data_in, uint32_t len)
{
    // search for pac address and bus
    if (i2c_iter == m_i2c_channels.end())
    {
//Serial.println("pac address not found");
        return false;
    }
// Serial.print("writing to pac address ");
// Serial.println(i2c_iter->second.first);
    byte error = 0;
    Wire.beginTransmission(i2c_iter->second.first);
    for (uint32_t b = 0; b < len; b++)
    {
        Wire.write(data_in[b]);
    }
    error = Wire.endTransmission();
// Serial.print("error = ");
// Serial.print(error);
// Serial.print(" on pac address ");
// Serial.println(i2c_iter->second.first);

    return (error == 0) ? true : false;
}

int I2CLib::read(byte channel, byte *data_out, uint32_t data_len)
{
    int out_len = 0;
    auto i2c_iter = get_i2c_address_(channel);
    if (i2c_iter == m_i2c_channels.end())
    {
//Serial.println("channel not found");
        return out_len;
    }

    if (i2c_iter->second.second == 1)
    {
//        out_len += read_(i2c_iter, PAC194X5X_VBUS1_AVG_ADDR, &data_out[0], 2);
        out_len += read_(i2c_iter,     PAC194X5X_VBUS1_ADDR, &data_out[0], 2);
        out_len += read_(i2c_iter,   PAC194X5X_VSENSE1_ADDR, &data_out[2], 2);
    }
    else if (i2c_iter->second.second == 2)
    {
//        out_len += read_(i2c_iter, PAC194X5X_VBUS2_AVG_ADDR, &data_out[0], 2);
        out_len += read_(i2c_iter,     PAC194X5X_VBUS2_ADDR, &data_out[0], 2);
        out_len += read_(i2c_iter,   PAC194X5X_VSENSE2_ADDR, &data_out[2], 2);
    }

// Serial.print("read ");
// Serial.print(out_len);
// Serial.print(" bytes from i2c address ");
// Serial.print(i2c_iter->second.first);
// Serial.print(" bus ");
// Serial.println(i2c_iter->second.second);
    return out_len;
}

I2C_MAP_ITERATOR I2CLib::get_i2c_address_(byte channel)
{
    // search for pac addres and bus
    I2C_MAP_ITERATOR i2c_iter = m_i2c_channels.find(channel);
    return i2c_iter;
}

int I2CLib::read_(I2C_MAP_ITERATOR i2c_iter, byte register_addr, byte *data_out, uint32_t len)
{
    if (i2c_iter == m_i2c_channels.end())
    {
//Serial.println("i2c address was not found");
        return 0;
    }
    // write to refresh register first
    byte refresh_data[] = { PAC194X5X_REFRESH_V_ADDR };
    if (write(i2c_iter->first, refresh_data, sizeof(refresh_data)) == false)
    {
//Serial.print("error writing address to pac ");
//Serial.println(i2c_iter->second.first);
        return 0;
    }
    // must wait before read
    delay(1);
    // send the register address to read from
    if (write(i2c_iter->first, &register_addr, 1) == true)
    {
        // now - read
        Wire.requestFrom(i2c_iter->second.first, len);
        while (Wire.available() < len);
        int k;
        for (k = 0; k < len; k++)
        {
            data_out[k] = (byte)Wire.read();
        }
        return k;
    }

    return 0;
}
