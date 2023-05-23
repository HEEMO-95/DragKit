'''
auther : heemo
this file contains mavlink commands and messages to be imported as functions
'''

import numpy as np
from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')  # mavproxy.py --out=udp:localhost:14551
master.wait_heartbeat()


def set_pos_local_ned(front:int, right:int, pos_alt:int, heading:float):

    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # refrance frame option
        (0b010111111000),  # control option
        front,  # meters in front of the airplane
        right,  # meters right of the airplane
        pos_alt,  # meters NEDown , -negative- value for ^upward^ height
        0,  # velocity in x direction type masking ingore this
        0,  # velocity in y direction
        0,  # velocity in z direction
        0, 0, 0, # accelartions (not supported)
        heading,  # yaw or heading in radians, 0 for front or north (depends on refrance frame)
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


def set_pos_glob(LAT:int, LON:int, ALT:float):

    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        10, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # refrance frame option
        (0b110111111000),  # control option blocks all and take pos int type
        LAT,  # Latitude
        LON ,  # Longitude
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
        action = 'stoping'
        pause_continue(0)

        while action == 'stoping':  # actions are done within inner loops
            nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  # action should have end conditions
            current_vx = float(nav.vx)
            current_vy = float(nav.vy)
            speed_vector= np.sqrt(current_vx**2 + current_vy **2)
            print(f'{speed_vector}, {action}')
            if speed_vector < 1 :  # inner while loop stop condition 
                action = 'stopped' # while loop shutdown line
                return action
            return action