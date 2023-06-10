from DragKit import *
import socket

HOST = 'localhost'
opject_PORT = 1995  # state socket port
position_PORT = 1996  # state socket port
shape_port = 1997  # state socket port

def mission():
    socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    socket.connect((HOST, opject_PORT))  # connect to opject identification server

    '''if recived a new opject detection message from server'''
    message = socket.recv(1024).decode('utf-8')
    i = 0
    while message == 'detected':
        a,b,c = do_stop()
        message = socket.recv(1024).decode('utf-8')
        if message == 'lost':
            i += 1
            a = go_to_local(b)

        elif i = 2:
            a = do_scan(port=opject_PORT)

        if a == 'found':
            break

    if a == 'stopped' or 'got_back' or 'found':
        ''' pass postion port to the align function which has a client socket to the position server'''
        a = align(position_PORT)

    if a == 'aligned':
        '''get shape data from shape server'''
        socket.connect((HOST, opject_PORT))  # connect to opject identification server
        message = socket.recv(1024).decode('utf-8')

    opject = shape()
    a = winch(opject)

    if a == 'done':
        # send opject info to server to append to visted list
        socket.send(f'{opject}'.encode('utf-8'))
        socket.close()
        resume()

while True:
    mission()