# بسم الله الرحمن الرحيم 
 # DragKit
Drag to infinity and beyond
![image](https://github.com/HEEMO-95/DragKit/assets/81169269/90da7923-fc9a-492f-97ae-e6eb0609a9d3)



DragKit python package made to allow our team in SAUS competion to program drones intelgently in few lines!

``` python
from DragKit import *

a, stop_point = do_stop()

if a == 'stopped':
    a = go_back(stop_point)

if a == 'got_back':
    do_scan()
```

With the ability to integrate computer vision to do precesion alignment.
DragKit GitHub:

# Background

Ardupilot:

Drone programming with python made easy by having the Ardupilot software running on the drone flight controller board.

Autopilot is a very capable autopilot software, including more than 700k lines of code, it can navigate the aircraft or any vehicle running the software with its advanced algorithms.

Ardupilot communicate with other programs via Mavlink messages, that can be transmitted or received by Ardupilot software, providing two way communication protocol allowing the user to read the vehicle data and send navigation commands, in conjunction with computer vision or other tools, it can become a very powerful tool.

# Pymavlink:
As the name suggests, the Pymavlink library allows to construct python scripts that uses the Mavlink protocol.

First, here is an example of connecting the drone to a ground station or on-board computer with Pymavlink on on-board computer:
``` python
from pymavlink import mavuti
Drone = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
``` 
after establishing a mavlink connection, we can start receiving aircraft status message
``` python
AHRS2 = Drone.recv_match(type='AHRS2', blocking=True)
heading = AHRS2.yaw
``` 
or mission sequance (waypoints) data message
``` python
mission = master.recv_match(type='MISSION_ITEM_REACHED', blocking=Tr
seq = mission.seq)eu
```
ing those reading constantly coming from the drone, allows to trigger actions at a desired point or state

Example of sending land command to aircraft
``` python
def takeoff_command(alt: int):
    Drone.mav.command_long_send(Drone.target_system, Drone.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0,0,0,0,0,0,  # empty target:
                            alt)  # altitude target:

takeoff_command(50)  # takeoff to 50 meter
``` 
as shown sending commands allows to specify action parameters with 'Enums' that explained in the Pymavlink and Arudpilot documents.

this is a very simple python script, but in order to create powerful script able to do intelligent stuff, you need to add logic, and condtions to meet based vehicle state

Here lays the power of DragKit package!

# DragKit

Actions:

DragKit is made by a list of 'actions', actions are a combination of multiple commands and messages in loops checking the aircraft state and provide conformation ubon completion, with the nesscary feedback data if any.

Action expample:

def do_stop()
``` python
        done = False
        pause_continue(0)
        nav = Drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        pos_x, pos_y, pos_z = float(nav.x), float(nav.y), float(nav.z)

        while not done:
            nav = Drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            current_vx = float(nav.vx)
            current_vy = float(nav.vy)
            speed_vector= np.sqrt(current_vx**2 + current_vy **2)
            print(f'{speed_vector}, {action}')

            if speed_vector < 1 : 
                action = 'stopped'
                done = True
                start_stop_point = (pos_x, pos_y, pos_z)
                return action, stop_point:
``` 
'do_stop' action is used when the aircraft is on route to a waypoint, the actions usually made of a setup, and a loop, the setup prepares the aircraft to take the action, while the loop keeps an eye of the vehicle state.

To resume the flight after stopping, here is another action example that can be run if desired action feedback is met.
``` python
def resume():
    flight_mode('AUTO')
    pause_continue(1)
    action = 'normal'
    return action
# no while loop
``` 
Actions arguments

Actions may have arguments, you can provide your own arugment to the action or leave it empty for defualt, see for example 'go_back' action :
``` python
def go_back(point=(0,0,20),yaw=0)

    #setup
    x , y , z = point[0], point[1], point[2]
    flight_mode('GUIDED')
    set_pos_local_ned(x,y,z,yaw)
    done = False

    #loop
    while not done:
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_vx, current_vy = float(nav.vx), float(nav.vy)
        speed_vector= np.sqrt(current_vx**2 + current_vy **2)

        if speed_vector > 1:
            mav= Drone.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            wp_dist = mav.wp_dist

            if wp_dist == 0 :
                done = True
                action = 'got_back'
            
    return action:
``` 
go_back defualt waypoint is the start waypoint ('0,0,0' in local frame), and the defualt yaw=0 means the aircraft will hold heading when returning back, 1 allow to yaw towards the next waypoint.

# Computer vision alignment action

With the aid of computer vision, a camera can see and identfy a land mark and its position on a picutre frame, and it import it to our script as a tuple, the aircraft will keep adjusting its vertical and horizental speeds according to how far is the position of that opject from our center of the picutre frame, until its completely centered or (0,0), thus the speeds.
``` python
The align action:

def align(steps: tuple):
    done = false
    flight_mode('GUIDED')

    while not done:
        try:
            x, y = steps[0], steps[1]
        except:
            done = True
            return = 'lost'

        AHRS2 = master.recv_match(type='AHRS2', blocking=True)
        heading = AHRS2.yaw

        error_relative_heading = np.arctan2(y, x)
        compined_heading = heading + error_relative_heading

        nav = Drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_vx, current_vy = float(nav.vx), float(nav.vy)
        speed_vector= np.sqrt(current_vx**2 + current_vy **2)

        step_x = K * (error_vector* np.cos(compined_heading))
        step_y = K * (error_vector* np.sin(compined_heading))

        step_vector = np.sqrt(step**2 + step **2)

        if speed_vector != step_vector :
            set_vel_glob(step_x, step_y)

        if abs(speed_vector - step_vector) = 0:
            done = True
            return = 'aligned'
``` 
its important to align the opject heading to the drone, with the heading of the drone itself to, as the speeds is adjusted in the local frame of the drone, and keep updating the opject postion, until it reaches zero.

and ofcouse, inform the user if the aircraft is completely allinged or the reading of our opject has lost.

DragKit are currently being devloped, actions areconstantly being added

feel free to check DarkKit GitHub for the latest update, providing a feedback or push requests are welcome.
