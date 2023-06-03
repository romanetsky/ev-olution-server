#include "spi_lib.h"

bool SpiLib::Init()
{
    bool rc = true;
    m_spi = new SPIClass(VSPI);
    m_spi->begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);

    pinMode(m_spi->pinSS(), OUTPUT);

    // make expanders initialization
    byte data[6];
    int data_len = sizeof(data);
    data[0] = 0x09;
    data[1] = 0xA5;
    data[2] = 0x09;
    data[3] = 0x55;
    data[4] = 0x09;
    data[5] = 0x55;
    write(data, data_len);
    data[0] = 0x0A;
    data[1] = 0x5A;
    data[2] = 0x0A;
    data[3] = 0x55;
    data[4] = 0x0A;
    data[5] = 0x55;
    write(data, data_len);
    data[0] = 0x0B;
    data[1] = 0x5A;
    data[2] = 0x0B;
    data[3] = 0x55;
    data[4] = 0x0B;
    data[5] = 0x55;
    write(data, data_len);
    data[0] = 0x0C;
    data[1] = 0x55;
    data[2] = 0x0C;
    data[3] = 0x55;
    data[4] = 0x0C;
    data[5] = 0x55;
    write(data, data_len);
    data[0] = 0x0D;
    data[1] = 0xAA;
    data[2] = 0x0D;
    data[3] = 0x55;
    data[4] = 0x0D;
    data[5] = 0x55;
    write(data, data_len);
    data[0] = 0x0E;
    data[1] = 0x55;
    data[2] = 0x0E;
    data[3] = 0x55;
    data[4] = 0x0E;
    data[5] = 0x55;
    write(data, data_len);
    data[0] = 0x0F;
    data[1] = 0x5A;
    data[2] = 0x0F;
    data[3] = 0x55;
    data[4] = 0x0B;
    data[5] = 0x55;
    write(data, data_len);
    data[0] = 0x04;
    data[1] = 0x01;
    data[2] = 0x04;
    data[3] = 0x01;
    data[4] = 0x04;
    data[5] = 0x01;
    write(data, data_len);

    return rc;
}

void SpiLib::spiEnable_()
{
    if (m_spi)
    {
        // pull SS low to prep other end for transfer
        m_spi->beginTransaction(SPISettings(m_spiClk, MSBFIRST, SPI_MODE0));
        digitalWrite(m_spi->pinSS(), LOW);
    }
}

void SpiLib::spiDisable_()
{
    if (m_spi)
    {
        // pull ss high to signify end of data transfer
        digitalWrite(m_spi->pinSS(), HIGH);
        m_spi->endTransaction();
    }
}

void SpiLib::write(byte *data_in, uint32_t len)
{
    spiEnable_();
    for (int k = 0; k < len; k++)
    {
        m_spi->transfer(data_in[k]);
    }
    spiDisable_();
}

int SpiLib::writeRead(byte *data_in, byte *data_out, uint32_t len)
{
    int out_size = 0;
    spiEnable_();
    for (int k = 0; k < len; k++)
    {
        data_out[out_size++] = m_spi->transfer(data_in[k]);
    }
    spiDisable_();

    return out_size;
}
