# -*- coding: utf-8 -*-
"""
Created on Sat Jun 17 18:31:56 2023

@author: roman
"""
import socket

# Set up the server's IP address and port
server_name = '192.168.4.1' #'my_evo_host'  # Replace with the server's IP address
server_port = 4556  # Replace with the server's port number

# Create a socket object
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.settimeout(0.01);

# Connect to the server
client.connect((server_name, server_port))
print("Connected to the server.")

# Send data to the server
data = 'Hello, server!'
client.sendall(data.encode())
print("Data sent to the server:", data)

# Receive data from the server
received_data = client.recv(1024).decode()
print("Data received from the server:", received_data)

# Close the socket connection
client.close()
print("Connection closed.")
