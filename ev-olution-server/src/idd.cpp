#include "idd.h"

bool idd_decode(IN byte *msg, IN int msg_len,
                OUT SerialHeader **serial_header,
                OUT BatchHeader **batch_header,
                OUT DataHeader **data_header)
{
    byte *data = nullptr;
    unsigned int data_size = 0;
    bool synched = false;
    int n = 0;
    int k;
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
                synched = true;
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
        //    // DEBUG: write back to HOST
        //    Serial.write((byte*)serial_header, sizeof(*serial_header));
        //    Serial.write((byte*)batch_header, sizeof(*batch_header));
        //    Serial.write((byte*)data_header, sizeof(*data_header));
        //    Serial.flush();
    }

    return synched;
}
