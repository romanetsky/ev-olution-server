#include <Arduino.h>
#include <chrono>
#include <esp32-hal-log.h>
//#include "driver/i2c.h"
//#include <Wire.h>
#include <SPI.h>

#define VSPI_CLK  SCK
#define VSPI_MISO MISO
#define VSPI_MOSI MOSI
#define VSPI_SS   SS

static const int spiClk = 1000000; // 1 MHz
SPIClass* vspi = NULL;
void spiWrite(SPIClass *spi, byte* data, uint32_t len);
int  spiRead (SPIClass *spi, byte*  out, uint32_t len);
void send_report(byte* data, int data_size);

byte msg_buffer[2048];
byte out_buffer[2048];

static const byte PREFIX[] = { 0xCA, 0xFE, 0xCA, 0xFE };
static const byte MAGICWORD[] = { 0xBA, 0xDA, 0xBA, 0xDA };

#pragma pack(push,1)
typedef struct _serialHeader {
  byte prefix[sizeof(PREFIX)]; // "CAFECAFE"
  byte opCode;    // opcode: 0x01...0xFF
  int  data_size; // message leng in bytes, excluding this header
  byte magic_word[sizeof(MAGICWORD)]; // "BADABADA"
} SerialHeader;
typedef struct _batchHeader {
  int nof_data_elements;
  byte magic_word[sizeof(MAGICWORD)]; // "BADABADA"
} BatchHeader;
typedef struct _dataHeader {
  int nof_elements;
  byte magic_word[sizeof(MAGICWORD)]; // "BADABADA"
} DataHeader;
#pragma pack(pop)

void setup() {
  // put your setup code here, to run once:
//  log_d("begin setup...");

  Serial.begin(115200);
  Serial.setTimeout(1);
  // i2c_config_t conf = {
  //   .mode = I2C_MODE_MASTER,
  //   .sda_io_num = 21,
  //   .scl_io_num = 22,
  //   .sda_pullup_en = GPIO_PULLUP_ENABLE,
  //   .scl_pullup_en = GPIO_PULLUP_ENABLE,
  //   .master.clk_speed = 100000,
  // };
  // esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
  // //Wire

  vspi = new SPIClass(VSPI);
  vspi->begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  pinMode(VSPI_SS, OUTPUT);

//  log_d("setup done...");
}

void loop() {
  // put your main code here, to run repeatedly:
//  log_i("start main loop...");

  while (!Serial.available());
  int msg_size = (int)Serial.readBytes(msg_buffer, sizeof(msg_buffer));

  auto start = std::chrono::system_clock::now();

  // search for the prefix "CAFECAFE"
  SerialHeader* serial_header = nullptr;
  BatchHeader* batch_header = nullptr;
  DataHeader* data_header = nullptr;
  int nof_batch_elements = 0;

  byte* data = nullptr;
  unsigned int data_size = 0;
  bool header_found = false;
  int n = 0;
  int k;
  for (k = 0; k < msg_size; k++)
  {
    if (msg_buffer[k] == PREFIX[n])
      ++n;
    else
      n = 0;
    if (n == (int)sizeof(PREFIX))
    {
      serial_header = (SerialHeader*)&msg_buffer[k + 1 - (int)sizeof(PREFIX)];
      if (serial_header->magic_word[0] == MAGICWORD[0] && serial_header->magic_word[1] == MAGICWORD[1] &&
          serial_header->magic_word[2] == MAGICWORD[2] && serial_header->magic_word[3] == MAGICWORD[3])
      {
        data = (byte*)(serial_header) + sizeof(*serial_header);
        header_found = true;
        break;
      }
      else
      {
        // bad magic word
        n = 0;
      }
    }
  }

  // get batch header and check it
  if (header_found)
  {
    batch_header = (BatchHeader*)data;
    if (batch_header->magic_word[0] == MAGICWORD[0] && batch_header->magic_word[1] == MAGICWORD[1] &&
        batch_header->magic_word[2] == MAGICWORD[2] && batch_header->magic_word[3] == MAGICWORD[3])
    {
      nof_batch_elements = batch_header->nof_data_elements;
    }
    if (nof_batch_elements > 0)
    {
      data_header = (DataHeader*)(data + sizeof(*batch_header));
      data = (byte*)(data_header) + sizeof(*data_header);
    }

//    // DEBUG: write back to HOST
//    Serial.write((byte*)serial_header, sizeof(*serial_header));
//    Serial.write((byte*)batch_header, sizeof(*batch_header));
//    Serial.write((byte*)data_header, sizeof(*data_header));
//    Serial.flush();
  }
  else
  {
    // error
    SerialHeader* serial_header_out = (SerialHeader*)out_buffer;
    memcpy(serial_header_out, serial_header, sizeof(*serial_header_out));
    serial_header_out->data_size = sizeof(BatchHeader);
    BatchHeader* batch_header_out = (BatchHeader*)((byte*)serial_header_out + sizeof(*serial_header_out));
    batch_header_out->nof_data_elements = 0;
    memcpy(batch_header_out, MAGICWORD, sizeof(batch_header_out));
    send_report(out_buffer, sizeof(*serial_header_out) + sizeof(*batch_header_out));
    return;
  }

  SerialHeader* serial_header_out = (SerialHeader*)&out_buffer[0];
  memcpy(serial_header_out, serial_header, sizeof(*serial_header_out));
  serial_header_out->data_size = sizeof(BatchHeader);
  BatchHeader* batch_header_out = (BatchHeader*)((byte*)serial_header_out + sizeof(*serial_header_out));
  memcpy(batch_header_out, batch_header, sizeof(*batch_header_out));
  DataHeader* data_header_out = (DataHeader*)((byte*)batch_header_out + sizeof(*batch_header_out));
  memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
  byte* data_out = (byte*)data_header_out + sizeof(*data_header_out);
  int out_size = 0;

  for (k = 0; k < nof_batch_elements; k++)
  {
    switch (serial_header->opCode)
    {
    case 0xEA:
      // SPI read
      spiRead(vspi, out_buffer, data_header->nof_elements);
      break;
    case 0x1E:
      // SPI write
      spiWrite(vspi, data, data_header->nof_elements);
      break;
    case 0xEE:
      // SPI write and read
      spiWrite(vspi, data, data_header->nof_elements);
      out_size = spiRead(vspi, data_out, data_header->nof_elements);
      break;
    default:
      break;
    }
    data_header = (DataHeader*)(data + data_header->nof_elements);
    data = (byte*)(data_header) + sizeof(*data_header);

    // update num of output elements
    data_header_out->nof_elements = out_size;

    data_header_out = (DataHeader*)(data_out + out_size);
    memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
    data_out += sizeof(*data_header_out);

    serial_header_out->data_size += sizeof(*data_header_out) + out_size;
  }
//  auto stop = std::chrono::system_clock::now();
//  auto difference = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

  // write back to HOST
  send_report(out_buffer, sizeof(*serial_header_out) + serial_header_out->data_size);
  // msg_size = out_size + sizeof(MAGICWORD); // includes size of the magic word
  // Serial.write((byte*)PREFIX, sizeof(PREFIX));
  // Serial.write((byte*)&msg_opcode, sizeof(msg_opcode));
  // Serial.write((byte*)&msg_size, sizeof(msg_size));
  // Serial.write((byte*)&difference, sizeof(difference));
  // Serial.write((char*)MAGICWORD, sizeof(MAGICWORD));
  // Serial.write(out_buffer, out_size);
  // Serial.write((char*)MAGICWORD, sizeof(MAGICWORD));
  // Serial.flush();

  // log_i("end main loop (elapsed time %f)...", difference);
}

void send_report(byte* data, int data_size)
{
  // int msg_size = data_size + sizeof(MAGICWORD); // includes size of the magic word
  // Serial.write((byte*)PREFIX, sizeof(PREFIX));
  // Serial.write((byte*)&msg_opcode, sizeof(msg_opcode));
  // Serial.write((byte*)&msg_size, sizeof(msg_size));
  // Serial.write((byte*)&elapsed_time, sizeof(elapsed_time));
  // Serial.write((char*)MAGICWORD, sizeof(MAGICWORD));
  Serial.write(data, data_size);
  //Serial.write((char*)MAGICWORD, sizeof(MAGICWORD));
  Serial.flush();
}

void spiWrite(SPIClass *spi, byte* data, uint32_t len) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer(data, len);
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
}

int spiRead(SPIClass *spi, byte* out, uint32_t len) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  int readBytes = 0;
  while (readBytes < len)
  {
    out[readBytes++] = spi->transfer(0x00);
  }
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();

  return readBytes;
}
