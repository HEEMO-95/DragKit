import socket

HOST = 'localhost'
PORT = 1995  # state socket port

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.connect((HOST, PORT))
print('connected')
message = socket.recv(1024).decode('utf-8')

for data in message:
    if data != '':
        data = message.split(',')
        seq = data[0]
        state = data[1]
        print(f'waypoint:{seq}, state:{state}')