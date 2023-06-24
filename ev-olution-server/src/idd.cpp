#include "idd.h"

byte idd_decode(IN byte *msg, IN int msg_len,
                OUT SerialHeader **serial_header,
                OUT BatchHeader **batch_header,
                OUT DataHeader **data_header)
{
    byte *data = nullptr;
    unsigned int data_size = 0;
    byte synched = OPCODE_ERROR_OK;
    int n = 0;
    int k;
    // sync on PREFIX
    for (k = 0; k < msg_len; k++)
    {
        if (msg[k] == PREFIX[n])
            ++n;
        else
            n = 0;
        if (n == (int)sizeof(PREFIX))
        {
            *serial_header = (SerialHeader *)&msg[k + 1 - (int)sizeof(PREFIX)];
            if ((*serial_header)->magic_word[0] == MAGICWORD[0] && (*serial_header)->magic_word[1] == MAGICWORD[1])
            {
                data = (byte *)(*serial_header) + sizeof(SerialHeader);
                synched = OPCODE_ERROR_OK;
                break;
            }
            else
            {
                // bad magic word
                n = 0;
                synched = OPCODE_ERROR_NOMAGIC;
            }
        }
        else
        {
            synched = OPCODE_ERROR_NOPREFIX;
        }
    }

    // check the expected data size
    if (synched == OPCODE_ERROR_OK)
    {
        if ((*serial_header)->data_size + sizeof(SerialHeader) > msg_len)
        {
            // expected data size is too long
            synched = OPCODE_ERROR_BADLENGTH;
        }
    }

    // check crc
    if (synched == OPCODE_ERROR_OK)
    {
        auto crc = crc16((const uint8_t*)(*serial_header), sizeof(SerialHeader) - sizeof(SerialHeader::crc));
        if (crc != (*serial_header)->crc)
        {
            synched = OPCODE_ERROR_CRC;
        }
    }

    // get batch header and check it
    if (synched)
    {
        *batch_header = (BatchHeader *)data;
        if ((*batch_header)->magic_word[0] == MAGICWORD[0] && (*batch_header)->magic_word[1] == MAGICWORD[1])
        {
            if ((*batch_header)->nof_data_elements > 0)
            {
                *data_header = (DataHeader *)(data + sizeof(BatchHeader));
                data = (byte *)(data_header) + sizeof(DataHeader);
            }
        }
        else
        {
            synched = OPCODE_ERROR_NOMAGIC;
        }
    }

    return synched;
}

void send_error(BaseCommunicator* comm, byte* tx_buffer, byte error_code)
{
    SerialHeader* serial_header_out = (SerialHeader*)tx_buffer;
    memcpy(serial_header_out->prefix, PREFIX, sizeof(PREFIX));
    serial_header_out->opCode = error_code;
    serial_header_out->data_size = 0;
    memcpy(serial_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
    serial_header_out->crc = crc16(tx_buffer, sizeof(SerialHeader) - sizeof(SerialHeader::crc));
    comm->write(tx_buffer, sizeof(SerialHeader));
}

uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; ++i) {
        crc ^= (data[i] << 8);
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
            crc &= 0xFFFF;
        }
    }

    return crc & 0xFFFF;
}
