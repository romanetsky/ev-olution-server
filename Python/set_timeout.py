# -*- coding: utf-8 -*-
"""
Created on Mon Jun 19 22:42:30 2023

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

arduino = serial.Serial(port='COM5', baudrate=115200, timeout=None)
time.sleep(1)
data = arduino.read_all()
arduino.flushInput()

timeout_sec = 10*60 # [] - is for reading only
data_bytes = np.array(timeout_sec, dtype='uint32').tobytes()
prefix = np.array([0xCA,0xFE], dtype='uint8').tobytes()
# opcode = 0xDA - set timeout value in secs
opcode = np.array([0xDA], dtype='uint8').tobytes()
magic_word = np.array([0xBA,0xDA], dtype='uint8').tobytes()
nof_batch_elements = np.array(1, dtype='uint32').tobytes()
nof_data_elements = np.array(len(data_bytes), dtype='uint32').tobytes()
data_size = np.array(
    [len(nof_batch_elements) + len(magic_word) +
     len(nof_data_elements) + len(magic_word) +
     len(data_bytes)], dtype='uint32').tobytes()

# serial header
packet = prefix
packet = packet + opcode
packet = packet + data_size
packet = packet + magic_word
packet = packet + np.array([crc16(packet)], dtype='uint16').tobytes()
# batch header
packet = packet + nof_batch_elements
packet = packet + magic_word
# data header
packet = packet + magic_word
packet = packet + nof_data_elements
# data
packet = packet + data_bytes

# send write request
arduino.write(packet)
# read the status
while arduino.in_waiting < 11:
    None
data_out_hdr = arduino.read(11) # read the header
in_data_size = struct.unpack('<I', data_out_hdr[3:7])[0]
while arduino.in_waiting < in_data_size:
    None
data_out = arduino.read_all()
print('==== STATUS MESSAGE ====')
print('header        : ', data_out_hdr.hex(':'))
print('batch header  : ', data_out[0:6].hex(':'))
print('data          : ');
print('  data hdr    : ', data_out[6:12].hex(':'))
print('  data        : ', data_out[12:].hex(':'))

arduino.close()
