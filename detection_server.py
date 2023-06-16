from AI import *
import cv2
import socket
import time
import numpy as np

def stripper(data):
    new_data = {}
    for k, v in data.items():
        if isinstance(v, dict):
            v = stripper(v)
        if not v in (u'', None, {}):
            new_data[k] = v
    return new_data

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


HOST = 'localhost'
PORT1 = 3014
PORT2 = 5016

objs_list = {
    'circle':
    {
        'green':{
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

#cap = cv2.VideoCapture('/dev/video0')
cap = cv2.VideoCapture('/home/jetson/Desktop/F1.mp4')
print('start capturing')
detector = Detector()
print('model is loaded')
cap.read()
server1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('server created')
server1.bind((HOST, PORT1))
server2.bind((HOST, PORT2))

server1.listen(5)
server2.listen(5)

connected = False
print('start')

connected = False
mav_socket, address = server1.accept()
connected = True
print(f'connected to {address}')
mode = 'ens'
while connected:
    ob = None
    objs = []
    if mode == 'ens':
        print('stsrt ens')
        while True:               
            ret, frame = cap.read()
            objs.clear()
            objs = detector.predict(frame=frame,stream=True)   
            if objs:
                print(len(objs),'> detected')
                dis = objs_dis(objs,[0,0])
                dis = list(dis)
                objs = [obj for _,obj in sorted(zip(dis,objs),key=lambda pair : pair[0])]
                for obj in objs:
                    col = color_detection(obj[1])
                    print(obj[0])
                    print(col)
                    temp = objs_list.get(obj[0],dict()).get(col,None)
                    if temp != None:
                        mav_socket.send('detected#'.encode('utf-8'))
                        mode = 'al'
                        break
                break
            else:
                print('no object detected, yet..')

    if mode == 'al': 
        ali_socket, address = server2.accept()
        state = None
        c=0         
        while True:
            ret, frame = cap.read()
            objs.clear()
            objs = detector.predict(frame=frame,stream=True)
            if objs:
                dis = objs_dis(objs,[0,0])
                dis = list(dis)
                objs = [obj for _,obj in sorted(zip(dis,objs),key=lambda pair : pair[0])]
                for obj in objs:
                    ob = obj
                    col = color_detection(ob[1])
                    print(ob[0])
                    print(col)
                    temp = objs_list.get(ob[0],dict()).get(col,None)
                    if temp != None:
                        c=0
                        msg = str(ob[-2])+','+str(ob[-1])+'#'
                        ali_socket.send(msg.encode('utf-8'))
                        break
                    else:
                        c=c+1
                        ali_socket.send(str('fuck#').encode('utf-8'))
                        break
            print('listen')
            state = ali_socket.recv(1024).decode('utf-8').split('#')
            for data in state:
                if data != '':
                    state = data
                    print(state)

            if state == 'aligned':
                mode = 'ad'
                break
            if c == 10:
                mode = 'ans'
                ali_socket.send(str('mother_fucker').encode('utf-8'))
                break

    if mode == 'ad':          
        time.sleep(2)
        color = color_detection(ob[1])
        charc = alphabetic_detection(ob[1])
        charc = 'A'
        shape,color,charc,idx = get_winch(ob[0],color,charc)
        mav_socket.send(str(idx).encode('utf-8'))
        del objs_list[shape][color][charc]
        objs_list = stripper(objs_list)
        mav_socket.recv(1024).decode('utf-8')
        mode == 'ens'

    if not objs_list:
        mav_socket.send("mission_done#".encode('utf-8'))
                    

    # seq,state = socket.recv(1024).decode('utf-8').split(',')
    # print(f'waypoint:{seq}, state:{state}')
