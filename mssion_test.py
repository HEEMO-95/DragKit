from DragKit import *
import socket

HOST = 'localhost'
PORT1 = 1995
PORT2 = 1416
mysocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
mysocket.connect((HOST, PORT1))

def mission():
    action = 'mission'
    
    '''if recived a new opject detection message from server'''
    message = mysocket.recv(1024).decode('utf-8')  # function will halt here until data arrvie
    if message == 'mission_done':
        flight_mode('RTL')

    if message == 'detected':  # if data arrived, check it and proceed
        action, break_start, break_end = do_stop()

    if action == 'stopped':
        ''' pass postion port to the align function which has a client socket to the ID_server'''
        action = align(PORT2)

    if action == 'aligned':
        '''get shape data from shape server'''
        message = mysocket.recv(1024).decode('utf-8')
        print(message)

while True:
    mission() 