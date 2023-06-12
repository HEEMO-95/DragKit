'''
author name : ibraheem
author email : ibraheem.zhrany@gmail.com
this file contains mavlink commands and messages to be imported as functions
and a compnations of commands called 'actions'
'''

import time
import socket
import numpy as np
from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')  # mavproxy.py --out=udp:localhost:14551
master.wait_heartbeat()
socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


def MAV_CMD_NAV_WAYPOINT(Latitude:int, Longitude:int ,Altitude: int):
    master.mav.command_long_send(master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
        0,  # Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
        5,  # Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
        0,  # 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
        np.nan, #Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
        Latitude,
        Longitude,
        Altitude) # m)
    

def set_pos_local(front:int, right:int, pos_alt:int, yaw:float=1):

    if yaw == 1:
        type_mask=(0b110111111000)
    else:
        type_mask=(0b010111111000)

    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # refrance frame option
        type_mask,  # control option
        front,  # meters in front of the airplane
        right,  # meters right of the airplane
        pos_alt ,  # meters NEDown , -negative- value for ^upward^ height
        0,  # velocity in x direction type masking ignore this
        0,  # velocity in y direction
        0,  # velocity in z direction
        0, 0, 0, # accelartions (not supported)
        0,  # yaw or heading in radians, 0 for front or north (depends on refrance frame)
        0  # yaw rate in rad/s
    ))


def set_vel_local_ned(vel_x:float, vel_y:float):

    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # refrance frame option
        (0b010111000111),  # control option
        0,  # meters in front of the airplane
        0,  # meters right of the airplane
        0,  # meters NEDown , -negative- value for ^upward^ height
        vel_x,  # velocity in x direction type masking ingore this
        vel_y,  # velocity in y direction
        0,  # velocity in z direction
        0, 0, 0, # accelartions (not supported)
        0,  # yaw or heading in radians, 0 for front or north (depends on refrance frame)
        0  # yaw rate in rad/s
    ))


def set_pos_glob(LAT:int, LON:int, ALT:float, yaw = 1):

    if yaw == 1:
        type_mask=(0b110111111000)

    else:
        type_mask=(0b010111111000)

    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        10, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # refrance frame option
        type_mask,  # control option blocks all and take pos int type
        LAT,  # Latitude
        LON,  # Longitude
        ALT,  # in meters, +postive+ for ^upward^ height
        0,  # velocity in x direction type masking ingore this
        0,  # velocity in y direction
        0,  # velocity in z direction
        0, 0, 0, # accelartions (not supported)
        0,  # yaw or heading in radians, 0 for front or north (depends on refrance frame)
        0  # yaw rate in rad/s
    ))


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


def pause_continue(pause0_continue1:int):
        master.mav.command_long_send(master.target_system, master.target_component,
                         mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE, 0,
                         pause0_continue1,  # 0: Pause & hold position. 1: Continue mission
                         0,0,0,0,0,0)


def override_pause(pause_continue:int, Yaw: int, LAT:int, LON:int, ALT:float):
        master.mav.command_long_send(master.target_system, master.target_component,
                         mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO, 0,
                         pause_continue,  # 0: Pause & hold a position. 1: Continue mission
                         3,  # MAV_GOTO_HOLD_AT_SPECIFIED_POSITION
                         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                         np.rad2deg(Yaw),  # Yaw deg
                         LAT,  # Latitude
                         LON ,  # Longitude
                         ALT)  # in meters, +postive+ for ^upward^ height


def flight_mode(mode: str):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    

def flight_mode_CMD(mode: str):
    mode_id = master.mode_mapping()[mode]
    master.mav.command_long_send(master.target_system, master.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mode_id,  # mode slsected
                            0,0,0,0,0,0)

def arm_command(arm: int):
    master.mav.command_long_send(master.target_system, master.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            arm,  # 1 for arm, 0 for disarm
                            0,0,0,0,0,0)


def takeoff_command(alt: int):
    master.mav.command_long_send(master.target_system, master.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0,0,0,0,0,0,
                            alt)  # altitude target
     

def land_command():
        master.mav.command_long_send(master.target_system, master.target_component,
                         mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
                         0,0,0,0,0,0,0)
        

def request_message_interval(message_id: int, frequency_hz: float):

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id,  # The MAVLink message ID
        1e6 / frequency_hz,  # The interval between two messages in microseconds.
        # set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0,  # Unused parameters
        0,  # Target address of message stream (if message has target address fields).
        # 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )


def do_stop():
        done = False
        pause_continue(0)
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        break_start = float(nav.x), float(nav.y), float(nav.z)
        while not done:
            nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            current_vx, current_vy = float(nav.vx) , float(nav.vy)
            speed_vector= np.sqrt(current_vx**2 + current_vy**2)
            print(f'{speed_vector}, {action}')
            if speed_vector < 0.5 : 
                action = 'stopped'
                done = True
                break_end = (nav.x, nav.y, nav.z)
                return action, break_start, break_end

      
def do_scan(port, scans = [(-5,-5),(5,-5),(5,5),(-5,5)], yaw = 0):
    i = 0
    action = 'do_scan'
    flight_mode('GUIDED')
    print(action)
    move = True
    nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  # action should have end conditions
    pos_x, pos_y, pos_z = float(nav.x), float(nav.y), float(nav.z)
    AHRS2 = master.recv_match(type='AHRS2', blocking=True)
    heading = AHRS2.yaw
    done = False

    HOST = 'localhost'
    socket.connect((HOST, port))
    print('scan connected')

    while not done:
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  # action should have end conditions
        current_vx, current_vy = float(nav.vx), float(nav.vy)
        speed_vector= np.sqrt(current_vx**2 + current_vy **2)
        message = socket.recv(1024).decode('utf-8')
           
        try:
            scan = scans[i]
        except:
            done = True
            action = 'scan_done'   # loop breaker
            return action
        
        for data in message:
            data = message.split(',')
            state = data[0]
            if state == 'found':
                done = True
                move = False
                action = 'scan_done'
                pause_continue(1)
                return action
        
        print(f'{speed_vector}, {action}, {i}')
        if  move :
            error = scan
            error_vector= np.sqrt(error[0]**2 + error[1]**2)
            error_relative_heading= np.arctan2(error[1], error[0])
            compined_heading = heading + error_relative_heading
            pos_x = pos_x + (error_vector* np.cos(compined_heading))
            pos_y = pos_y + (error_vector* np.sin(compined_heading))
            print(f' x = {pos_x}, y = {pos_y}')
            set_pos_local(pos_x,pos_y,pos_z, yaw)
            move = False

        if speed_vector > 1:
            mav = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            wp_dist = mav.wp_dist
            if wp_dist == 0 :
                i+=1
                move = True


def go_to_local(point=(0,0,20),yaw=1):

    x , y , z = point[0], point[1], point[2] * -1
    flight_mode('GUIDED')
    set_pos_local(x,y,z,yaw)
    done = False
    o_time = time.time()
    while not done:
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_vx, current_vy = float(nav.vx), float(nav.vy)
        speed_vector= np.sqrt(current_vx**2 + current_vy **2)

        if speed_vector > 1 or time.time() - o_time > 5:
            mav= master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            wp_dist = mav.wp_dist
            o_time = time.time()
            if wp_dist == 0 :
                done = True
                action = 'got_back'
                return action


def go_to_glob(point=(0,0,-20),yaw=1):

    x , y , z = int(point[0]*1e7), int(point[1]*1e7), point[2]
    flight_mode('GUIDED')
    set_pos_glob(x,y,z,yaw)
    done = False
    o_time = time.time()
    while not done:
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_vx, current_vy = float(nav.vx), float(nav.vy)
        speed_vector= np.sqrt(current_vx**2 + current_vy **2)

        if speed_vector > 1 or time.time() - o_time > 5:
            mav= master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            wp_dist = mav.wp_dist
            o_time = time.time()
            if wp_dist == 0 :
                done = True
                action = 'got_back'
                return action


def align(port):
    done = False
    flight_mode('GUIDED')
    K = 0.001
    HOST = 'localhost'
    PORT = port  # state socket port
    socket.connect((HOST, PORT))
    print('alingment connected')
    
    while not done:
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        speed_vector= np.sqrt(nav.vx**2 + nav.vy**2)
        Attitude = master.recv_match(type='AHRS2', blocking=True)
        heading, pitch, roll = Attitude.yaw, Attitude.pitch, Attitude.roll
        
        try:
            x,y = socket.recv(1024).decode('utf-8').split(',')
            error_vector = np.sqrt(x**2 + y**2)
            
        except:
            done = True
            return 'lost'
        
        else:

            error_relative_heading = np.arctan2(y, x)
            compined_heading = heading + error_relative_heading
            step_x = np.cos(roll) * K * (error_vector * np.cos(compined_heading))
            step_y = np.cos(pitch) * K * (error_vector * np.sin(compined_heading))
            step_vector = np.sqrt(step_x**2 + step_y**2)

            if speed_vector != step_vector :
                set_vel_glob(step_x, step_y)

            if abs(speed_vector - step_vector) <= 0.1:
                socket.close()
                done = True
                return 'aligned'



def resume():
    flight_mode('AUTO')
    pause_continue(1)
    action = 'normal'
    return action
