#include <Arduino.h>
#include <SPI.h>

#pragma once

#define SPI_SCK     18   // SPI clock pin
#define SPI_MISO    19   // SPI MISO (Master-In Slave-Out) pin
#define SPI_MOSI    23   // SPI MOSI (Master-Out Slave-In) pin
#define SPI_CS      5    // SPI chip select pin

class SpiLib
{
public:
    bool Init();
    void write(byte *data_in, uint32_t len);
    int  writeRead(byte *data_in, byte *data_out, uint32_t len);

private:
    void spiEnable_();
    void spiDisable_();

private:
    const int m_spiClk = 1000000; // 1 MHz
    SPIClass *m_spi = nullptr;
};
