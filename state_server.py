from DragKit import *
import socket

HOST = 'localhost'
PORT = 1995

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(5)
connected = False

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_REACHED,2)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2)
state_dict = {1:'ground',2:'air',3:'taking_off',4:'landing'}


def send_data():
    connected = False
    state_socket, address = server.accept()
    connected = True
    print(f'connected to {address}')

    while connected:

        item_reached = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
        seq = item_reached.seq
        state_msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
        state_num = state_msg.landed_state

        state = state_dict[state_num]
        data = (f'{seq},{state}')
        try:
            state_socket.send(f"{data}".encode('utf-8'))
        except:
            connected = False


while True:
    send_data()