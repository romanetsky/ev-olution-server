#include <Arduino.h>
#include <chrono>
#include <esp32-hal-log.h>
//#include "driver/i2c.h"
//#include <Wire.h>
#include <SPI.h>

static const int spiClk = 1000000; // 1 MHz
SPIClass* vspi = NULL;
void spiWrite(SPIClass *spi, byte* data, uint32_t len);
int  spiRead (SPIClass *spi, byte*  out, uint32_t len);
void send_report(int msg_opcode, int64_t elapsed_time, byte* data, int data_size);

byte msg_buffer[2048];
byte out_buffer[2048];

static const byte PREFIX[] = { 0xCA, 0xFE, 0xCA, 0xFE };
static const byte MAGICWORD[] = { 0xBA, 0xDA, 0xBA, 0xDA };

#pragma pack(push,1)
typedef struct {
  byte prefix[sizeof(PREFIX)]; // "CAFECAFE"
  byte opCode;    // opcode: 0x01...0xFF
  int  data_size; // message leng in bytes, excluding this header
  byte sufix[sizeof(MAGICWORD)]; // "BADABADA"
} SerialHeader;

typedef struct {
  int nof_elements;
} BatchHeader;

typedef struct {
  byte magic_word[sizeof(MAGICWORD)]; // "BADABADA"
} SerialFooter;
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
 vspi->begin();

//  log_d("setup done...");
}

void loop() {
  // put your main code here, to run repeatedly:
//  log_i("start main loop...");

  while (!Serial.available());
  int msg_size = (int)Serial.readBytes(msg_buffer, sizeof(msg_buffer));

  auto start = std::chrono::system_clock::now();

  // search for the prefix "CAFECAFE"
  SerialHeader* hdr = nullptr;
  byte* data = nullptr;
  unsigned int data_size = 0;
  byte msg_opcode = 0;
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
      hdr = (SerialHeader*)&msg_buffer[k + 1 - (int)sizeof(PREFIX)];
      if (hdr->sufix[0] == MAGICWORD[0] && hdr->sufix[1] == MAGICWORD[1] &&
          hdr->sufix[2] == MAGICWORD[2] && hdr->sufix[3] == MAGICWORD[3])
      {
        data = &msg_buffer[k + sizeof(SerialHeader) + 1];
        data_size = hdr->data_size - sizeof(MAGICWORD); // size includes the MAGICWORD
        msg_opcode = hdr->opCode;
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

  // DEBUG: write back to HOST
  if (header_found)
  {
    // Serial.write((byte*)hdr->prefix, sizeof(hdr->prefix));
    // Serial.write((byte*)&msg_opcode, sizeof(msg_opcode));
    // Serial.write((byte*)&data_size, sizeof(data_size));
    // Serial.write((char*)hdr->sufix, sizeof(hdr->sufix));
    // Serial.flush();
  }
  else
  {
    // error
    send_report(msg_opcode, 0, out_buffer, 0);
    return;
  }

  int out_size = 0;
  switch (msg_opcode)
  {
  case 0xEA:
    // SPI read
    spiRead(vspi, out_buffer, data_size);
    break;
  case 0x1E:
    // SPI write
    spiWrite(vspi, data, data_size);
    break;
  case 0xEE:
    // SPI write and read
    spiWrite(vspi, data, data_size);
    out_size = spiRead(vspi, out_buffer, data_size);
    break;
  default:
    break;
  }

  auto stop = std::chrono::system_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

  // write back to HOST
  send_report(msg_opcode, difference, out_buffer, out_size);
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

void send_report(int msg_opcode, int64_t elapsed_time, byte* data, int data_size)
{
  int msg_size = data_size + sizeof(MAGICWORD); // includes size of the magic word
  Serial.write((byte*)PREFIX, sizeof(PREFIX));
  Serial.write((byte*)&msg_opcode, sizeof(msg_opcode));
  Serial.write((byte*)&msg_size, sizeof(msg_size));
  Serial.write((byte*)&elapsed_time, sizeof(elapsed_time));
  Serial.write((char*)MAGICWORD, sizeof(MAGICWORD));
  Serial.write(data, data_size);
  Serial.write((char*)MAGICWORD, sizeof(MAGICWORD));
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
