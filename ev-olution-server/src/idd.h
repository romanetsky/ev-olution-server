#include <Arduino.h>

#pragma once
#pragma pack(push,1)

#define IN
#define OUT
#define INOUT

static const byte PREFIX[]      = { 0xCA, 0xFE };
static const byte MAGICWORD[]   = { 0xBA, 0xDA };
static const byte ERROR_OPCODE  = 0xEF;

typedef struct _serialHeader {
  byte prefix[sizeof(PREFIX)]; // "CAFE"
  byte opCode;    // opcode: 0x01...0xEF
  int  data_size; // message leng in bytes, excluding this header
  byte magic_word[sizeof(MAGICWORD)]; // "BADA"
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

bool idd_decode(byte *msg, int msg_len,
                SerialHeader **serial_header,
                BatchHeader **batch_header,
                DataHeader **data_header);
