import socket
from AI import *
import cv2

HOST = 'localhost'
mav_PORT = 1995  # state socket port
server_port = 1996
print('connecting to mavlink server......')
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.connect((HOST, mav_PORT))
print('connected to mavlink server')

print('create a server')
server.bind((HOST, server_port))
mav_server, address = server.accept()
print('created')


seq_tar = 3
visited_obj = set()
cap = cv2.VideoCapture('/dev/video0')
detector = Detector()



while True:
    seq,state = socket.recv(1024).decode('utf-8').split(',')
    print(f'waypoint:{seq}, state:{state}')
    if seq == seq_tar or state == 'winched':
        frame = cap.read()
        objs = detector.predict(frame=frame)
        if objs:
            for obj in objs:
                if obj not in visited_obj:
                    mav_server.send(str(['detected',obj[-1],obj[-2]]).encode('utf-8'))
    if state == 'aligned':
       color = color_detection(obj[1])
       charac = alphabetic_detection(obj[1])
       print(obj+[color]+[charac])
       visited_obj.add(obj+[color]+[charac])
    
    if len(visited_obj) == 5:
        mav_server.send("mission_done".encode('utf-8'))
                

