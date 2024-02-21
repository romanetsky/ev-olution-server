# -*- coding: utf-8 -*-
"""
Created on Sun Apr 23 18:45:15 2023

@author: roman
"""
import serial
import struct
import numpy as np
import time

def crc16(data):
    crc = 0xFFFF

    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF

    return crc & 0xFFFF

arduino = serial.Serial(port='COM8', baudrate=115200, timeout=None)
time.sleep(1)
data = arduino.read_all()
arduino.flushInput()

data1 = [
         [0x44,0x01,0x44,0x01,0x44,0x01],
         [0x4C,0x02,0x4C,0x02,0x4C,0x02],
         [0x54,0x03,0x54,0x03,0x54,0x03],
         [0x5C,0x04,0x5C,0x04,0x5C,0x04],
         # [0x58,0x05,0x58,0x05,0x58,0x05],
         [0x0F,0x01,0x0F,0x01,0x0F,0x01]
         ]

# data1 = [
#          [68,1,68,0,68,0],
#          [76,64,76,1,76,0],
#          [84,0,84,0,84,0],
#          [92,0,92,0,92,0],
#          # [0x58,0x01,0x58,0x01,0x58,0x01],
#          #[0x0F,0x01,0x0F,0x01,0x0F,0x01]
#          ]

for i in range(0,1):
    for j in range(0,1):
        # for x in range(0, len(data1)):
        #     for y in range(1,len(data1[x]),2):
        #         data1[x][y] = j
        
        data_bytes = np.array(data1, dtype='uint8')
        nx, ny = data_bytes.shape;
        nof_batch_elements = np.array(nx, dtype='uint32').tobytes()
        nof_elements = np.array(ny, dtype='uint32').tobytes()
        prefix = np.array([0xCA,0xFE], dtype='uint8').tobytes()
        # opcode = 0x6A - read; 0x65 - write; 0x60 - write & read
        opcode = np.array([0x65], dtype='uint8').tobytes()
        magic_word = np.array([0xBA,0xDA], dtype='uint8').tobytes()
        data_size = np.array(
            [len(nof_batch_elements) + len(magic_word) +    # batch header
             nx * (len(nof_elements) + len(magic_word) +    # data headers
             len(data_bytes[0]))                            # datum
            ],
            dtype='uint32').tobytes()
        
        # serial header
        packet = prefix
        packet = packet + opcode
        packet = packet + data_size
        packet = packet + magic_word
        packet = packet + np.array([crc16(packet)], dtype='uint16').tobytes()
        # batch header
        packet = packet + nof_batch_elements
        packet = packet + magic_word
        for k in range(0, nx):
            # data header
            packet = packet + magic_word
            packet = packet + nof_elements
            # data
            packet = packet + data_bytes[k,:].tobytes()
        
        start_time = time.time_ns()
        # send write request
        arduino.write(packet)
        # read the write status
        while arduino.in_waiting < 11:
            None
        data_out_hdr = arduino.read(11) # read the header
        in_data_size = struct.unpack('<I', data_out_hdr[3:7])[0]
        while arduino.in_waiting < in_data_size:
            None
        data_out = arduino.read_all()
        stop_time = time.time_ns()
        print('==== WRITE REQUEST STATUS ====')
        print('header        : ', data_out_hdr.hex(':'))
        print('batch header  : ', data_out[0:6].hex(':'))
        print('data          : ', data_out[6:].hex(':'))
        print('overall command time: ', (stop_time - start_time)/1000000, ' msec')

# arduino.close()
