#include <Arduino.h>
#include <SPI.h>

#pragma once

#define DEF_PORT_STATE_4_11     0x44
#define DEF_PORT_STATE_12_19    0x4C
#define DEF_PORT_STATE_20_27    0x54
#define DEF_PORT_STATE_28_31    0x5C

#define SPI_SCK                 18   // SPI clock pin
#define SPI_MISO                19   // SPI MISO (Master-In Slave-Out) pin
#define SPI_MOSI                23   // SPI MOSI (Master-Out Slave-In) pin
#define SPI_CS                  5    // SPI chip select pin
#define SPI_CLOCK_RATE          1000000 // 1MHz

class SpiLib
{
public:
    bool Init();
    void Reset();
    void ConfigurePorts();
    void write(byte *data_in, uint32_t len);
    int  read(byte *data_in, byte *data_out, uint32_t len);
    int  writeread(byte *data_in, byte *data_out, uint32_t len);

private:
    void spiEnable_();
    void spiDisable_();

private:
    const int m_spiClk = SPI_CLOCK_RATE; // 1 MHz
    SPIClass *m_spi = nullptr;
};
