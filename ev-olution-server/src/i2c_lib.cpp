#include "i2c_lib.h"
//#include "PAC194x_5x.h"

bool I2CLib::Init()
{
    bool rc = true;

 //   rc = m_ina219.begin();

    // Wire.begin(I2C_SDA, I2C_SCL, I2C_CLOCK_RATE);

    // // scan for I2C devices
    // byte error, address;
    // int nDevices;
    // Serial.println("Scanning...");
    // nDevices = 0;
    // for (address = 1; address < 127; address++)
    // {
    //     Wire.beginTransmission(address);
    //     error = Wire.endTransmission();
    //     if (error == 0)
    //     {
    //         Serial.print("I2C device found at address 0x");
    //         if (address < 16)
    //         {
    //             Serial.print("0");
    //         }
    //         Serial.println(address, HEX);
    //
    //         if (address == I2C_PAC_ADDRESS)
    //         {
    //             m_pac_found = true;
    //             Serial.print("PAC device found at expected address 0x");
    //             Serial.print(I2C_PAC_ADDRESS, HEX);
    //             Serial.println("Stop scanning I2C devices...");
    //             break;
    //         }
    //         nDevices++;
    //     }
    //     else if (error == 4)
    //     {
    //         Serial.print("Unknow error at address 0x");
    //         if (address < 16)
    //         {
    //             Serial.print("0");
    //         }
    //         Serial.println(address, HEX);
    //     }
    // }
    // if (nDevices == 0)
    // {
    //     Serial.println("No I2C devices found");
    // }
    // else
    // {
    //     Serial.println("done");
    // }

    // m_config = {
    //   .mode = I2C_MODE_MASTER,
    //   .sda_io_num = 21,
    //   .scl_io_num = 22,
    //   .sda_pullup_en = GPIO_PULLUP_ENABLE,
    //   .scl_pullup_en = GPIO_PULLUP_ENABLE,
    //   .master.clk_speed = I2C_CLOCK_RATE,
    // };
    //
    // esp_err_t err = i2c_param_config(I2C_NUM_0, &m_config);
        
    return rc;
}
