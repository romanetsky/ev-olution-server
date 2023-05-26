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
print(data)

data1 = np.array([0x44,0x03,0x44,0x03,0x44,0x03], dtype='uint8').tobytes()
# data1 = np.array([9,165,9,85,9,85], dtype='uint8').tobytes()
# data2 = np.array([10,90,10,85,10,85], dtype='uint8').tobytes()
# data3 = np.array([11,90,11,85,11,85], dtype='uint8').tobytes()
# data4 = np.array([12,85,12,85,12,85], dtype='uint8').tobytes()
# data5 = np.array([13,170,13,85,13,85], dtype='uint8').tobytes()
# data6 = np.array([14,85,14,85,14,85], dtype='uint8').tobytes()
# data7 = np.array([15,90,15,85,15,85], dtype='uint8').tobytes()
# data8 = np.array([4,1,4,1,4,1], dtype='uint8').tobytes()
nof_batch_elements = np.array(1, dtype='uint32').tobytes()
nof_elements = np.array(len(data1), dtype='uint32').tobytes()
prefix = np.array([0xCA,0xFE], dtype='uint8').tobytes()
opcode = np.array([0x1E], dtype='uint8').tobytes()
magic_word = np.array([0xBA,0xDA], dtype='uint8').tobytes()
data_size = np.array(
    [len(nof_batch_elements) + len(magic_word) + # batch header
     len(nof_elements) + len(magic_word) + # data header 1
     len(data1) + len(magic_word)  # data 1
     # len(nof_elements) + len(magic_word) +  # data header 2
     # len(data2) + len(magic_word) + # data 2
     # len(nof_elements) + len(magic_word) + # data header 3
     # len(data3) + len(magic_word) + # data 3
     # len(nof_elements) + len(magic_word) + # data header 4
     # len(data4) + len(magic_word) + # data 4
     # len(nof_elements) + len(magic_word) + # data header 5
     # len(data5) + len(magic_word) + # data 5
     # len(nof_elements) + len(magic_word) + # data header 6
     # len(data6) + len(magic_word) + # data 6
     # len(nof_elements) + len(magic_word) + # data header 7
     # len(data7) + len(magic_word) + # data 7
     # len(nof_elements) + len(magic_word) + # data header 8
     # len(data8) + len(magic_word) # data 8
    ],
    dtype='uint32').tobytes()

# serial header
packet = prefix
packet = packet + opcode
packet = packet + data_size
packet = packet + magic_word
# batch header
packet = packet + nof_batch_elements
packet = packet + magic_word
# data header 1
packet = packet + nof_elements
packet = packet + magic_word
# data 1
packet = packet + data1
# # data header 2
# packet = packet + nof_elements
# packet = packet + magic_word
# # data 2
# packet = packet + data2
# # data header 3
# packet = packet + nof_elements
# packet = packet + magic_word
# # data 3
# packet = packet + data3
# # data header 4
# packet = packet + nof_elements
# packet = packet + magic_word
# # data 4
# packet = packet + data4
# # data header 5
# packet = packet + nof_elements
# packet = packet + magic_word
# # data 5
# packet = packet + data5
# # data header 6
# packet = packet + nof_elements
# packet = packet + magic_word
# # data 6
# packet = packet + data6
# # data header 7
# packet = packet + nof_elements
# packet = packet + magic_word
# # data 7
# packet = packet + data7
# # data header 8
# packet = packet + nof_elements
# packet = packet + magic_word
# # data 8
# packet = packet + data8
 

start_time = time.time_ns()
arduino.write(packet)
while arduino.in_waiting < 7:
    None
data_out_hdr = arduino.read(7) # read the header
in_data_size = struct.unpack('<I', data_out_hdr[3:7])[0]
while arduino.in_waiting < in_data_size:
    None
data_out = arduino.read_all()
stop_time = time.time_ns()
print('header: ', data_out_hdr.hex(':'))
print('data  : ', data_out.hex(':'))

# while len(data) == 0:
#     data = arduino.read_all()
# in_prefix = struct.unpack('<2s', data[0:2])[0]
# in_opcode = struct.unpack('<c', data[2:3])[0]
# in_data_size = struct.unpack('<I', data[3:7])[0]
# in_magic_word = struct.unpack('<2s', data[7:9])[0]
# print('hdr.prefix\t\t: ', in_prefix)
# print('hdr.opcode\t\t: ', in_opcode)
# print('hdr.data_size\t: ', in_data_size)
# print('hdr.magic_word\t: ', in_magic_word)

# in_batch_elements = struct.unpack('<I', data[13:17])[0]
# in_magic_word = struct.unpack('<4s', data[17:21])[0]
# print('\tbtch_hdr.batch_elements\t: ', in_batch_elements)
# print('\tbtch_hdr.magic_word\t\t: ', in_magic_word)
# offset = 21
# for k in range(0,in_batch_elements):
#     in_data_elements = struct.unpack('<I', data[offset:offset+4])[0]
#     offset = offset + 4
#     in_magic_word = struct.unpack('<4s', data[offset:offset+4])[0]
#     offset = offset + 4
#     if in_data_elements > 0:
#         in_data = struct.unpack('<4s', data[offset:offset+in_data_elements])[0]
#         offset = offset + in_data_elements
#     else:
#         in_data = None
#     print('\t\tdata_hdr.data_elements\t: ', in_data_elements)
#     print('\t\tdata_hdr.magic_word\t\t: ', in_magic_word)
#     print('\t\tdata_hdr.data\t\t\t: ', in_data)

print('overall command time: ', (stop_time - start_time)/1000000, ' msec')
arduino.close()

# time.sleep(1)
# data = arduino.read_all()
# print(data)
# data = arduino.read_all()
# print(data)
# arduino.close()
