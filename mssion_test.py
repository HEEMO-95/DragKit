from DragKit import *
import socket

HOST = 'localhost'
PORT = 1995
mysocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
mysocket.connect((HOST, PORT))

def mission():
    action = 'mission'
    
    '''if recived a new opject detection message from server'''
    message = mysocket.recv(1024).decode('utf-8')  # function will halt here until data arrvie
    if message = 'mission_done':
        flight_mode('RTL')

    if message == 'detected':  # if data arrived, check it and proceed
        action, break_start, break_end = do_stop()

    if action == 'stopped':
        ''' pass postion port to the align function which has a client socket to the ID_server'''
        action = align(PORT)

    if action == 'aligned':
        '''get shape data from shape server'''
        mysocket.connect((HOST, PORT))  # connect to opject identification server


    message = mysocket.recv(1024).decode('utf-8')
    winched = winch(message)

    mysocket.send('align'.encode('utf-8'))
    if winched == True: 
        resume()

while True:
    mission() 