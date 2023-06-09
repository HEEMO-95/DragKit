from DragKit import *
from AI import object_detection
import serial

def m1(seq_ind):
    """
    args:
        seq_ind: the sequence number of the last waypoint
    return:
        True: if the seq_ind is reached

    A function will do the folowing:
    1- go to the waypoints
    2- return True if all waypoints are visited
    """

    ''' IBRAHEEM:
            didnt quite understand the part above but here is the thing :
            cannot resume flightplan from a specfic waypoint to specific waypoint
            flight plan sequance are rigid

            we can let the drone fly a pre-defined route and stop and allign when a new opject is identified
            much simpler and less room for unexpectations'''

    while True:
        mission = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
        seq = mission.seq
        if seq == seq_ind:
            break
    return True


def m2(detect_obj,objs,vid_cap):
    """
    args:
        detect_obj: a function that detect the objects
    return:
        True: If the Object is detected

    A function that do the folowing:
    1- go to object detection area

            '''IBRAHEEM:
    "1- go to object detection area" < cannot
                better wait for mission squance to reach the detection area, detection area waypoint must be known'''

    2- looking for objects
    3- return the True if an object is detected
    """
    #if the drone in obj detection area

    #else go to obj detection area

    #end of go to obj detection area code

    #end if

    
    while True:
        frame=vid_cap.read()
        obj = detect_obj(frame)
        if obj != []:
            for o in obj:
                if o not in objs:
                    return True

def m3(detect_obj,vid_cap):
    """
    args:
        detect_obj: a function to detect the object
    returns:
        True: if the drone is above the object
    A function will do the folowing:
    1- align to object location
    2- correction of object info
    """
    while True:
        frame=vid_cap.read()
        # code for alignment
        obj = detect_obj(frame)
        x,y=obj.xy
        #end of alignment
        break
    obj = detect_obj(frame)
    return obj

def m4(slot_num,arduino):
    """
    A function will do the folowing:
    1- Air drop
    """
    arduino.write(bytes(str(slot_num),encoding='utf-8'))
    return True

def m5():
    """
    A function do the folowing:
    1- return to home
    """
    pass

wp_seq = None
objs = []
detect_obj = object_detection
num_bottels = 3
c = 0
arduino = serial.Serial("/dev/ttyUSB0", 57600)
slot_map = {('red','circle','A'):'1'}
while True:
    if c == 0:
        m1(wp_seq)

    m2(detect_obj,objs)
    objs.append(m3(detect_obj)) 
    slot_num = slot_map[(objs[-1].color,objs[-1].shape,objs[-1].alphabet)]
    m4(slot_num,arduino)
    c = c + 1
    
    if c == num_bottels:
        m5()
        if len(objs) == 5:
            break
        else:
            c = 0
            continue

# landing


