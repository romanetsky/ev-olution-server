# -*- coding: utf-8 -*-
"""
Created on Fri Jun 16 20:28:25 2023

@author: roman
"""

import socket

Hostname = 'localhost'
PortNumber = 6100
Buffer = 512
ServerAddress = (Hostname, PortNumber)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(ServerAddress)

while True:
    print('The client is connected to the server')
    DataStr = input('Enter data to send to the server: ')
    if not DataStr:
        print('The client has entered nothing; hence the connection to the server is closed')
        client_socket.close()
        break
    else:
        client_socket.send(DataStr)
        ServerData = client_socket.recv(Buffer)
        if not ServerData:
            print('The server has sent nothing')
        else:            
            print('The server has sent this data string: ', ServerData)
            break
