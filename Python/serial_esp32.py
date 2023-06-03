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
arduino.flushInput()

# data1 = np.array([[4,1,4,1,4,1],
#                  [0,1,2,3,4,5]], dtype='uint8')
data1 = np.array([[68,1,68,0,68,2],
                  [76,0,76,0,76,80],
                  [84,0,84,0,84,0],
                  [92,0,92,0,92,0]], dtype='uint8')

data1 = np.array([[68,1,68,0,68,132],
                  [76,0,76,0,76,64],
                  [84,0,84,0,84,0],
                  [92,0,92,0,92,0]], dtype='uint8')
nx, ny = data1.shape;
nof_batch_elements = np.array(nx, dtype='uint32').tobytes()
nof_elements = np.array(ny, dtype='uint32').tobytes()
prefix = np.array([0xCA,0xFE], dtype='uint8').tobytes()
opcode = np.array([0x2E], dtype='uint8').tobytes()
magic_word = np.array([0xBA,0xDA], dtype='uint8').tobytes()
data_size = np.array(
    [len(nof_batch_elements) + len(magic_word) +    # batch header
     nx*(len(nof_elements) + len(magic_word) +      # data headers
     len(data1) + len(magic_word))                  # datum
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
for k in range(0, nx):
    # data header
    packet = packet + magic_word
    packet = packet + nof_elements
    # data
    packet = packet + data1[k,:].tobytes()

arduino.flushInput()
start_time = time.time_ns()
arduino.write(packet)
while arduino.in_waiting < 9:
    None
data_out_hdr = arduino.read(9) # read the header
in_data_size = struct.unpack('<I', data_out_hdr[3:7])[0]
while arduino.in_waiting < in_data_size:
    None
data_out = arduino.read_all()
stop_time = time.time_ns()
print('header      : ', data_out_hdr.hex(':'))
print('batch header: ', data_out[0:6].hex(':'))
print('data        : ', data_out[6:].hex(':'))
print('overall command time: ', (stop_time - start_time)/1000000, ' msec')

arduino.close()

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


# time.sleep(1)
# data = arduino.read_all()
# print(data)
# data = arduino.read_all()
# print(data)
# arduino.close()
