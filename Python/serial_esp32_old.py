# -*- coding: utf-8 -*-
"""
Created on Sun Apr 23 18:45:15 2023

@author: roman
"""
import serial
import struct
import numpy as np
import time

# def write_read(x):
#     arduino.write(bytes(x, 'utf-8'))
#     arduino.flush()
#     data = arduino.read_all()
#     return data

arduino = serial.Serial(port='COM5', baudrate=115200, timeout=None)
time.sleep(1)
data = arduino.read_all()

data = np.array([9,165,9,85,9,85], dtype='uint8').tobytes()
# data = np.array([], dtype='uint8').tobytes()
prefix = np.array([0xCA,0xFE,0xCA,0xFE], dtype='uint8').tobytes()
opcode = np.array([0x1E], dtype='uint8').tobytes()
sufix = np.array([0xBA,0xDA,0xBA,0xDA], dtype='uint8').tobytes()
data_size = np.array([len(data)+len(sufix)], dtype='uint32').tobytes()

packet = prefix
packet = packet + opcode
packet = packet + data_size
packet = packet + sufix
packet = packet + data
packet = packet + sufix

start_time = time.time_ns()
arduino.write(packet)
data = arduino.read_until(size=25)
stop_time = time.time_ns()
# while len(data) == 0:
#     data = arduino.read_all()
print(data)
in_prefix = struct.unpack('<4s', data[0:4])[0]
in_opcode = struct.unpack('<c', data[4:5])[0]
in_data_size = struct.unpack('<I', data[5:9])[0]
in_duration = struct.unpack('<Q', data[9:17])[0]
in_sufix = struct.unpack('<4s', data[17:21])[0]
print('in prefix: ', in_prefix)
print('in opcode: ', in_opcode)
print('in data size: ', in_data_size)
print('in duration: ', in_duration)
print('in sufix: ', in_sufix)
print('overall esp time: ', struct.unpack('<q', data[9:17])[0]/1000, ' msec')
print('overall command time: ', (stop_time - start_time)/1000000, ' msec')
arduino.close()

# time.sleep(1)
# data = arduino.read_all()
# print(data)
# data = arduino.read_all()
# print(data)
# arduino.close()
