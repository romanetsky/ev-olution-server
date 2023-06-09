#include <Arduino.h>
#include <chrono>
#include <esp32-hal-log.h>
//#include <Wire.h>>
#include "idd.h"
#include "spi_lib.h"
#include "i2c_lib.h"

#define POWER_ON_PIN        4
#define LED_STATUS          27
#define SerialBaudRate      115200
#define SerialTimeOutMS     1
#define RCV_BUFFER_SIZE     512

byte rx_buffer[RCV_BUFFER_SIZE];
byte tx_buffer[RCV_BUFFER_SIZE];

SpiLib spi_lib;
I2CLib i2c_lib;

void setup() {
  // put your setup code here, to run once:
//  log_d("begin setup...");

  Serial.begin(SerialBaudRate);
  Serial.setTimeout(SerialTimeOutMS);

  // power ON
  pinMode(POWER_ON_PIN, OUTPUT);
  digitalWrite(POWER_ON_PIN, HIGH);
  delay(1);

  // I2C library init
//  i2c_lib.Init();
  // SPI library init
  spi_lib.Init();

// m_spi = new SPIClass(VSPI);
// m_spi->begin();
// pinMode(5, OUTPUT);
//     byte data[6];
//     int len = sizeof(data);
//     data[0] = 0x09;
//     data[1] = 0xA5;
//     data[2] = 0x09;
//     data[3] = 0x55;
//     data[4] = 0x09;
//     data[5] = 0x55;
//     m_spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//     digitalWrite(5, LOW); // pull SS low to prep other end for transfer
//     for (int k = 0; k < len; k++)
//     {
//         m_spi->transfer(data[k]);
//     }
//     digitalWrite(5, HIGH); // pull ss high to signify end of data transfer
//     m_spi->endTransaction();
//     delay(10);
//     data[0] = 11;
//     data[1] = 90;
//     data[2] = 11;
//     data[3] = 85;
//     data[4] = 11;
//     data[5] = 85;
//     m_spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//     digitalWrite(5, LOW); // pull SS low to prep other end for transfer
//     for (int k = 0; k < len; k++)
//     {
//         m_spi->transfer(data[k]);
//     }
//     digitalWrite(5, HIGH); // pull ss high to signify end of data transfer
//     m_spi->endTransaction();
//     delay(10);
//     data[0] = 12;
//     data[1] = 85;
//     data[2] = 12;
//     data[3] = 85;
//     data[4] = 12;
//     data[5] = 85;
//     m_spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//     digitalWrite(5, LOW); // pull SS low to prep other end for transfer
//     for (int k = 0; k < len; k++)
//     {
//         m_spi->transfer(data[k]);
//     }
//     digitalWrite(5, HIGH); // pull ss high to signify end of data transfer
//     m_spi->endTransaction();
//     delay(10);
//     data[0] = 13;
//     data[1] = 170;
//     data[2] = 13;
//     data[3] = 85;
//     data[4] = 13;
//     data[5] = 85;
//     m_spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//     digitalWrite(5, LOW); // pull SS low to prep other end for transfer
//     for (int k = 0; k < len; k++)
//     {
//         m_spi->transfer(data[k]);
//     }
//     digitalWrite(5, HIGH); // pull ss high to signify end of data transfer
//     m_spi->endTransaction();
//     delay(10);
//     data[0] = 14;
//     data[1] = 85;
//     data[2] = 14;
//     data[3] = 85;
//     data[4] = 14;
//     data[5] = 85;
//     m_spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//     digitalWrite(5, LOW); // pull SS low to prep other end for transfer
//     for (int k = 0; k < len; k++)
//     {
//         m_spi->transfer(data[k]);
//     }
//     digitalWrite(5, HIGH); // pull ss high to signify end of data transfer
//     m_spi->endTransaction();
//     delay(10);
//     data[0] = 15;
//     data[1] = 90;
//     data[2] = 15;
//     data[3] = 85;
//     data[4] = 11;
//     data[5] = 85;
//     m_spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//     digitalWrite(5, LOW); // pull SS low to prep other end for transfer
//     for (int k = 0; k < len; k++)
//     {
//         m_spi->transfer(data[k]);
//     }
//     digitalWrite(5, HIGH); // pull ss high to signify end of data transfer
//     m_spi->endTransaction();
//     delay(10);
//     data[0] = 4;
//     data[1] = 1;
//     data[2] = 4;
//     data[3] = 1;
//     data[4] = 4;
//     data[5] = 1;
//     m_spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//     digitalWrite(5, LOW); // pull SS low to prep other end for transfer
//     for (int k = 0; k < len; k++)
//     {
//         m_spi->transfer(data[k]);
//     }
//     digitalWrite(5, HIGH); // pull ss high to signify end of data transfer
//     m_spi->endTransaction();
//     delay(10);

  // light the led
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, HIGH);

  //  log_d("setup done...");
}

void loop() {
//  log_d("start main loop...");

  auto start = std::chrono::system_clock::now();

  // wait for the serial header
  while (Serial.available() < sizeof(SerialHeader)) delay(1);

  int msg_size = (int)Serial.readBytes(rx_buffer, RCV_BUFFER_SIZE);
  // if not enough data, return 
  if (rx_buffer == nullptr || msg_size < sizeof(SerialHeader))
  {
    return;
  }

  // search for the prefix "CAFE"
  SerialHeader *serial_header = nullptr;
  BatchHeader *batch_header = nullptr;
  DataHeader *data_header = nullptr;

  // sync on serial header
  byte error_code = idd_decode(
    IN rx_buffer, IN msg_size,
    OUT &serial_header, OUT &batch_header, OUT &data_header
  );

  if (error_code != OPCODE_ERROR_OK)
  {
    // send error
    send_error(tx_buffer, error_code);
    return;
  }

  SerialHeader* serial_header_out = (SerialHeader*)&tx_buffer[0];
  memcpy(serial_header_out, serial_header, sizeof(*serial_header_out));
  serial_header_out->data_size = sizeof(BatchHeader);
  BatchHeader* batch_header_out = (BatchHeader*)((byte*)serial_header_out + sizeof(*serial_header_out));
  memcpy(batch_header_out, batch_header, sizeof(*batch_header_out));
  batch_header_out->nof_data_elements = 0; // it will be updated lated, in accordance with message type 
  DataHeader* data_header_out = (DataHeader*)((byte*)batch_header_out + sizeof(*batch_header_out));
  memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
  byte* data_out = (byte*)data_header_out + sizeof(*data_header_out);

  int out_size = 0;
  byte* data = (byte*)(data_header) + sizeof(*data_header);
  for (int k = 0; k < batch_header->nof_data_elements; k++)
  {
    switch (serial_header->opCode)
    {
    case OPCODE_SPI_READ:
      // SPI read
      out_size = spi_lib.read(data, data_out, data_header->nof_elements);

      // increment nof batch elements
      batch_header_out->nof_data_elements += 1;

      // update num of output elements
      data_header_out->nof_elements = out_size;
      // advance data header ptr
      data_header_out = (DataHeader *)(data_out + out_size);
      memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
      data_out += sizeof(*data_header_out) + out_size;
      // update total size
      serial_header_out->data_size += sizeof(*data_header_out) + out_size;
      break;
    case OPCODE_SPI_WRITE:
      // SPI write
      spi_lib.write(data, data_header->nof_elements);
      break;
    default:
      break;
    }

    //
    data_header = (DataHeader*)(data + data_header->nof_elements);
    data = (byte*)(data_header) + sizeof(*data_header);
  }

  auto stop = std::chrono::system_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

  // write back to HOST
  Serial.write(tx_buffer, sizeof(*serial_header_out) + serial_header_out->data_size);
  Serial.flush();

//  log_d("end main loop (elapsed time %f)...", difference);
}
