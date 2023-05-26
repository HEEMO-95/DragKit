'''
auther : heemo
this file contains mavlink commands and messages requests in logical loops
'''

import numpy as np
from DragKit import *

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_REACHED,2)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2)

steps = [1.5, 1, 0.5]   # (simulation)
scans = [(-5,-5),(5,-5),(5,5),(-5,5)]  # (simulation)
action = 'normal'
active = False
x=1  # waypoint action
xx=350  # waypoint distance action (simulation)
while True:
    msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
    state = msg.landed_state

    '''
    DATA STREAM LINE
    data = (x_error, y_error, action_override:str)
    '''

    if (active == False) and (state == 2):
        mav = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        wp_dist = mav.wp_dist

        mission = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
        seq = mission.seq

        if (seq == x) and (wp_dist < xx > xx-10):
            action = 'do_stop'
            active = True

    if action == 'do_stop' :  # when to trigger the action
        action = 'stoping'
        pause_continue(0)
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  # action should have end conditions
        pos_x, pos_y, pos_z = float(nav.x), float(nav.y), float(nav.z)

        while action == 'stoping':  # actions are done within inner loops
            nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  # action should have end conditions
            current_vx , current_vy = float(nav.vx), float(nav.vy)
            speed_vector= np.sqrt(current_vx**2 + current_vy **2)
            print(f'{speed_vector}, {action}')
            current_vx, current_vy = float(nav.vx), float(nav.vy)
            if speed_vector < 1 :  # inner while loop stop condition 
                action = 'stopped' # while loop shutdown line


    if action == 'stopped':
        action = 'alli' # loop trigger
        flight_mode('GUIDED')
        i = 0
        while action == 'alli':
            nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  # action should have end conditions
            current_vx, current_vy = float(nav.vx), float(nav.vy)
            speed_vector= np.sqrt(current_vx**2 + current_vy **2)
            try:
                step = steps[i]  # (simulation)
            except:
                action = 'lost'   # loop breaker

            step_vector = np.sqrt(step**2 + step **2)
            print(f'{speed_vector}, {action}, {i}')
            if speed_vector != step_vector :
                set_vel_glob(step,step)
                print('commanding movement')
            if abs(speed_vector-step_vector) < 0.05:
                i+=1

    if action == 'resume':
        flight_mode('AUTO')
        pause_continue(1)
        print(action)
        action = 'normal'
        print(action)
        break

    if action == 'lost':
        print(action)
        action = 'go_back'
        print(action)
        flight_mode('GUIDED')
        set_pos_local_ned(pos_x,pos_y,pos_z,0)
        while action == 'go_back':
            nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  # action should have end conditions
            current_vx, current_vy = float(nav.vx), float(nav.vy)
            speed_vector= np.sqrt(current_vx**2 + current_vy **2)
            if speed_vector > 1:
                mav = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
                wp_dist = mav.wp_dist
                if wp_dist == 0 :
                    action = 'still_lost'


    if action == 'still_lost':
        print(action)
        i=0
        action = 'do_scan'
        flight_mode('GUIDED')
        print(action)
        move = True
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  # action should have end conditions
        pos_x, pos_y, pos_z = float(nav.x), float(nav.y), float(nav.z)
        
        while action == 'do_scan':
            nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  # action should have end conditions
            current_vx, current_vy = float(nav.vx), float(nav.vy)
            speed_vector= np.sqrt(current_vx**2 + current_vy **2)

            AHRS2 = master.recv_match(type='AHRS2', blocking=True)
            heading = AHRS2.yaw

            try:
                scan = scans[i]

            except:
                action = 'resume'   # loop breaker

            print(f'{speed_vector}, {action}, {i}')

            if move == True:
                error = scan
                error_vector= np.sqrt(error[0]**2 + error[1]**2)

                error_relative_heading= np.arctan2(error[1], error[0])

                AHRS2 = master.recv_match(type='AHRS2', blocking=True)
                heading = AHRS2.yaw

                compined_heading = heading + error_relative_heading

                pos_x = pos_x + (error_vector* np.cos(compined_heading))
                pos_y = pos_y + (error_vector* np.sin(compined_heading))
                print(f' x = {pos_x}, y = {pos_y}')

                set_pos_local_ned(pos_x,pos_y,pos_z,0)
                move = False

            if speed_vector > 1:
                mav = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
                wp_dist = mav.wp_dist
                if wp_dist == 0 :
                    i+=1
                    move = True
