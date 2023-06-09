#include <Arduino.h>

#pragma once
#pragma pack(push,1)

#define IN
#define OUT
#define INOUT

static const byte OPCODE_SPI_READ       = 0x6A;
static const byte OPCODE_SPI_WRITE      = 0x65;
static const byte OPCODE_I2C_READ       = 0x9A;
static const byte OPCODE_I2C_WRITE      = 0x95;
static const byte OPCODE_ERROR_OK       = 0xE0;
static const byte OPCODE_ERROR_CRC      = 0xEC;
static const byte OPCODE_ERROR_NOPREFIX = 0xEF;
static const byte OPCODE_ERROR_NOMAGIC  = 0xEA;
static const byte OPCODE_ERROR_NOSYNC   = 0xED;
static const byte OPCODE_ERROR_BADLENGTH= 0xEB;
static const byte PREFIX[]          = { 0xCA, 0xFE };
static const byte MAGICWORD[]       = { 0xBA, 0xDA };

typedef struct _serialHeader {
  byte      prefix[sizeof(PREFIX)]; // "CAFE"
  byte      opCode;     // opcode
  int       data_size;  // message length in bytes, excluding this header
  byte      magic_word[sizeof(MAGICWORD)]; // "BADA"
  uint16_t  crc;        // header crc
} SerialHeader;
typedef struct _batchHeader {
  int nof_data_elements;
  byte magic_word[sizeof(MAGICWORD)]; // "BADA"
} BatchHeader;
typedef struct _dataHeader {
  byte magic_word[sizeof(MAGICWORD)]; // "BADA"
  int nof_elements;
} DataHeader;

#pragma pack(pop)

byte idd_decode(byte *msg, int msg_len,
                SerialHeader **serial_header,
                BatchHeader **batch_header,
                DataHeader **data_header);

uint16_t crc16(const uint8_t *data, size_t length);

void send_error(byte* tx_buffer, byte error_code);
