from AI import *
import cv2
from DragKit import *
import socket
import time
import numpy as np

HOST = 'localhost'
PORT1 = 1995
PORT2 = 1416

objs_list = {
    'circle':
    {
        'red':{
            'A':1
            },
        'blue':{

        }
        },
    'squer':
    {
        'red':{
            'A':2
            }
        },
    'hexagon':
    {
        'red':{
            'A':3
            }
        },
    'octagon':
    {
        'red':{
            'A':4
            }
        },
    'triangle':
    {
        'red':{
            'A':5
            }
        }
               }

def get_winch(shape,color,charc):
    color_dict = objs_list.get(shape,None)
    if color_dict == None:
        return 'fuck','fuck','fuck','fuck'
    
    charc_dict = color_dict.get(color,None)

    if charc_dict == None:
        charc_dict = color_dict.get(list(color_dict.keys())[0])
        idx = charc_dict.get(list(charc_dict.keys())[0])
        charc = list(charc_dict.keys())[0]
        color = list(color_dict.keys())[0]
        return shape,color,charc,idx
    
    idx = charc_dict.get(charc,None)

    if idx == None:
        charc = list(charc_dict.keys())[0]
        idx = charc_dict.get(list(charc_dict.keys())[0])
        return shape,color,charc,idx
    
    return shape,color,charc,idx

def objs_dis(objs,center):
    arr = np.array([[obj[-2],obj[-1]] for obj in objs])
    c = np.array(center)
    diff = np.sqrt(np.sum((arr**2) - (c**2),axis=1))
    return diff

cap = cv2.VideoCapture('/dev/video0')
detector = Detector()


server1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server1.bind((HOST, PORT1))
server2.bind((HOST, PORT2))

server1.listen(5)
server2.listen(5)

connected = False

while True:
    connected = False
    mav_socket, address = server1.accept()
    connected = True
    print(f'connected to {address}')
    mode = 'ens'
    while connected:
        ob = None
        try:
            if mode == 'ens':
                while True:               
                    frame = cap.read()
                    objs = detector.predict(frame=frame)                 
                    xc,yc = (int(frame.shape[0]/2) , int(frame.shape[1]/2))
                    if objs:
                        dis = objs_dis(objs)
                        dis = list(dis)
                        print(dis)
                        objs = [obj for _,obj in sorted(zip(dis,objs),key=lambda pair : pair[0])]
                        for obj in objs:
                            c = color_detection(obj[1])
                            temp = objs_list.get(obj[0],dict()).get(c,None)
                            if temp != None:
                                mav_socket.send('detected#'.encode('utf-8'))
                                mode = 'al'
                                break
                        break

            if mode == 'al': 
                ali_socket, address = server2.accept()
                state = None
                c=0         
                while True:
                    frame = cap.read()
                    objs = detector.predict(frame=frame)
                    if objs:
                        dis = objs_dis(objs)
                        dis = list(dis)
                        print(dis)
                        objs = [obj for _,obj in sorted(zip(dis,objs),key=lambda pair : pair[0])]
                        for obj in objs:
                            ob = obj
                            c = color_detection(ob[1])
                            temp = objs_list.get(ob[0],dict()).get(c,None)
                            if temp != None:
                                c=0
                                ali_socket.send(str([ob[-2],ob[-1]]).encode('utf-8'))
                            else:
                                c=c+1
                                ali_socket.send(str('fuck').encode('utf-8'))

                    state = ali_socket.recv(1024).decode('utf-8').split('#')
                    for data in state:
                        if data != '':
                            state = data
                            break

                    if state == 'aligned':
                        mode = 'ad'
                        break
                    if c == 10:
                        mode = 'ans'
                        ali_socket.send(str('mother_fuckern ').encode('utf-8'))
                        break

            if mode == 'ad':          
                time.sleep(2)
                color = color_detection(ob[1])
                # charc = alphabetic_detection(ob[1])
                charc = 'A'
                shape,color,charc,idx = get_winch(ob[0],color,charc)
                mav_socket.send(str(idx).encode('utf-8'))
                del objs_list[shape][color][charc]
                mav_socket.recv(1024).decode('utf-8')
                mode == 'ens'

            if not objs_list:
                mav_socket.send("mission_done#".encode('utf-8'))
                        
        except:
            connected = False


    # seq,state = socket.recv(1024).decode('utf-8').split(',')
    # print(f'waypoint:{seq}, state:{state}')