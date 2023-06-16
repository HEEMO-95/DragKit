import socket
import time
import numpy as np

from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # mavproxy.py --out=udp:localhost:14551
master.wait_heartbeat()

print('connected to mav')

def set_vel_glob(vel_x:float, vel_y:float):
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        10, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # refrance frame option
        (0b010111000111),  # control option blocks all and takes vel float type, and sets yaw rate 0 rad/s 
        0,  # Latitude
        0 ,  # Longitude
        0,  # in meters, +postive+ for ^upward^ height
        vel_x,  # velocity in x direction type masking ingore this
        vel_y,  # velocity in y direction
        0,  # velocity in z direction
        0, 0, 0, # accelartions (not supported)
        0,  # yaw or heading in radians, 0 for front or north (depends on refrance frame)
        0  # yaw rate in rad/s , 0 for no yaw during movment
    ))

def flight_mode(mode: str):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)


def align(port):
    done = False
    flight_mode('GUIDED')
    K = 0.005
    HOST = 'localhost'
    PORT = port  # state socket port
    mysocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mysocket.connect((HOST, PORT))
    print('alingment connected')
    
    while not done:
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        speed_vector= np.sqrt(nav.vx**2 + nav.vy**2)
        Attitude = master.recv_match(type='AHRS2', blocking=True)
        heading, pitch, roll = Attitude.yaw, Attitude.pitch, Attitude.roll

        try:
            msg = mysocket.recv(1024).decode('utf-8').split('#')
            for data in msg:
                if data != '':
                    msg = data
            print(msg)
            
            if msg == 'fuck':
                mysocket.send('align#'.encode('utf-8'))
                continue

            msg = msg.split(',')
            x, y = int(float(msg[0])),int(float(msg[1]))
            mysocket.send('align#'.encode('utf-8'))
            error_vector = np.sqrt(x**2 + y**2)

        except Exception as e:
            print(e)
            print('lost')
            done = True
            mysocket.close()
            return 'lost'
        
        else:
            
            error_relative_heading = np.arctan2(x, y)
            compined_heading = heading + error_relative_heading
            step_x = K * (error_vector * np.cos(compined_heading))
            step_y = K * (error_vector * np.sin(compined_heading))
            step_vector = np.sqrt(step_x**2 + step_y**2)
            print('control', speed_vector,step_vector)
            if speed_vector != step_vector :
                set_vel_glob(step_x, step_y)

            if abs(speed_vector - step_vector) < 0.1 and speed_vector < 0.5:
                print('aligned',abs(speed_vector - step_vector))
                set_vel_glob(0, 0)
                done = True
                mysocket.send('aligned#'.encode('utf-8'))
                mysocket.close()
                return 'aligned'


HOST = 'localhost'
PORT1 = 3014
PORT2 = 5016

mysocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
mysocket.connect((HOST, PORT1))
print('connected')
done = False

message = mysocket.recv(1024).decode('utf-8').split('#')
for data in message:
    if data != '':
        message = data
        
print(message)
if message == 'detected':
    a = align(PORT2)
    if a == 'aligned':
        message = mysocket.recv(1024).decode('utf-8')
        if message == '6':
            print(message)
            mysocket.send('confirmed#'.encode('utf-8'))