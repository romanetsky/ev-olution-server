# -*- coding: utf-8 -*-
"""
Created on Mon Jun 19 22:42:30 2023

@author: roman
"""
import serial
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

timeout_sec = 6
data_bytes = np.array(timeout_sec, dtype='uint32').tobytes()
prefix = np.array([0xCA,0xFE], dtype='uint8').tobytes()
# opcode = 0xDA - set timeout value in secs
opcode = np.array([0xDA], dtype='uint8').tobytes()
magic_word = np.array([0xBA,0xDA], dtype='uint8').tobytes()
nof_batch_elements = np.array(1, dtype='uint32').tobytes()
data_size = np.array(
    [len(nof_batch_elements) + len(magic_word) +
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
# data
packet = packet + data_bytes

# send write request
arduino.write(packet)

# arduino.close()
