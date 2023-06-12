from DragKit import *
import socket

HOST = 'localhost'
PORT = 50007
socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.connect((HOST, PORT))

def mission():
    
    '''if recived a new opject detection message from server'''

    message = socket.recv(1024).decode('utf-8')

    if message == 'detected':
        action, break_start, break_end = do_stop()
        socket.close()

    if action == 'stopped':
        ''' pass postion port to the align function which has a client socket to the position server'''
        align = align(PORT)

    if align == 'aligned':
        '''get shape data from shape server'''
        socket.connect((HOST, PORT))  # connect to opject identification server
        message = socket.recv(1024).decode('utf-8')

    winched = winch(message)
    if winched == True:
        resume()

while True:
    mission()