from DragKit import *

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
    while True:
        mission = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
        seq = mission.seq
        if seq == seq_ind:
            break
    return True


def m2(detect_obj):
    """
    args:
        detect_obj: a function that detect the objects
    return:
        obj: the detected object

    A function that do the folowing:
    1- go to object detection area
    2- looking for objects
    3- return the True if an object is detected
    """
    #if the drone in obj detection area return True

    #else go to obj detection area

    #end of go to obj detection area code

    #end if

    
    while True:
        frame=None
        obj = detect_obj(frame)
        if obj != []:
            break
    return True

def m3(detect_obj):
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
        frame=None
        # code for alignment
        obj = detect_obj(frame)
        x,y=obj.xy
        #end of alignment
        break
    obj = detect_obj(frame)
    return obj

def m4():
    """
    
    """
    pass

def m5():
    """
    A function do the folowing:
    1- return to home
    """
    pass

wp_seq = None
take_off = None
detect_obj = None
objs = []
num_bottels = 5
c = 0
while True:

    if c == 0:
        m1(wp_seq)
    
    m2(detect_obj)
    objs.append(m3(detect_obj))   
    m4()
    c = c + 1
    
    if c == num_bottels:
        m5()
        if len(objs) == 5:
            break
        else:
            c = 0
            continue
# landing
