 # DragKit

![image](https://github.com/HEEMO-95/DragKit/assets/81169269/90da7923-fc9a-492f-97ae-e6eb0609a9d3)


DragKit python package made to allow our team in SAUS competition to program drones intelligently in few lines!

``` python
from DragKit import *

a, stop_point = do_stop()

if a == 'stopped':
    a = go_back(stop_point)

if a == 'got_back':
    do_scan()
```

With the ability to implement computer vision for [precision alignment.](https://github.com/HEEMO-95/DragKit#computer-vision-alignment-action)




# Background

 ## Ardupilot: 

Drone programming with python made easy by having the Ardupilot software running on the drone flight controller board.

Autopilot is a very capable autopilot software, including more than 700k lines of code, it can navigate the aircraft or any vehicle running the software with its advanced algorithms.

Ardupilot communicate with other programs via Mavlink messages, that can be transmitted or received by Ardupilot software, providing two way communication protocol allowing the user to read the vehicle data and send navigation commands, in conjunction with computer vision or other tools, it can become a very powerful tool.

## Pymavlink:
As the name suggests, the Pymavlink library allows to construct python scripts that uses the Mavlink protocol.

First, lets take a look at an example of connecting the drone to a ground station or on-board computer with Pymavlink:
``` python
from pymavlink import mavuti
Drone = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
``` 
after establishing a mavlink connection, we can start receiving aircraft status message
``` python
AHRS2 = Drone.recv_match(type='AHRS2', blocking=True)
heading = AHRS2.yaw
``` 
or mission sequence (waypoints) data message:
``` python
mission = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
seq = mission.seq
```
with readings constantly coming from the drone, it allows to trigger actions at a desired point or state.

Example of sending takeoff command to aircraft
``` python
def takeoff_command(alt: int):
    Drone.mav.command_long_send(Drone.target_system, Drone.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0,0,0,0,0,0,  # non-used enums :
                            alt)  # altitude target:

takeoff_command(50)  # takeoff to 50 meter
``` 
as shown sending commands allows to specify action parameters with 'Enums' that explained in the Pymavlink and Arudpilot documents.

this is a very simple python script, but in order to create powerful script able to do intelligent stuff, you need to add logic, and conditions to meet a vehicle state

Here lays the power of DragKit package!

# DragKit Actions:

DragKit.py is made by a list of 'actions', actions are a combination of multiple commands and messages in loops checking the aircraft state and provide conformation upon completion, with the necessary feedback data if there was a need of any.

Action example:
``` python
def do_stop():
#  setup
    done = False
    pause_continue(0)
    nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    start_point = float(nav.x), float(nav.y), float(nav.z)
#  void
    while not done:
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_vx, current_vy = float(nav.vx) , float(nav.vy)
        speed_vector= np.sqrt(current_vx**2 + current_vy**2)
        if speed_vector < 0.5 : 
            action = 'stopped'
            done = True
            stop_point = (nav.x, nav.y, nav.v)
            return action, start_point, stop_point
``` 
'do_stop' action is used when the aircraft is on route to a waypoint, the actions usually made of a setup, and a loop, the setup prepares the aircraft to take the action, while the loop keeps an eye of the vehicle state.

To resume the flight after stopping, here is another action example that can be run if desired action feedback is met.
``` python
def resume():
    flight_mode('AUTO')
    pause_continue(1)
    action = 'normal'
    return action
   
a , start_point , stop_point = do_stop()

if a == 'stopped'
    resume()
``` 
## Actions arguments

Actions may have arguments, you can provide your own arguments to the action or leave it empty for default, see for example 'go_back' action:
``` python
def go_back(point=(0,0,50),yaw=0):

    #  setup
    x , y , z = point[0], point[1], point[2]
    flight_mode('GUIDED')
    set_pos_local_ned(x,y,z,yaw)
    done = False
    o_time = time.time()

    #  void
    while not done:
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_vx, current_vy = float(nav.vx), float(nav.vy)
        speed_vector= np.sqrt(current_vx**2 + current_vy **2)

        if speed_vector > 1 or time.time() - o_time > 5:
            mav= master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            wp_dist = mav.wp_dist

            if wp_dist == 0 :
                done = True
                action = 'got_back'
                return action
               
               
a , start_point , stop_point = do_stop() 

if a == 'stopped'
    go_back(start_point, yaw=1)
``` 
go_back default way point is the takeoff waypoint ('0,0,0' in local frame), and the default yaw=0 means the aircraft will hold heading when returning back, 1 allow to yaw towards the next waypoint, so the above the script will order the aircraft to stop, after it done braking and completely stopped, it will turn and head to the point when the stop action was initiated.

# Computer vision alignment action

With the aid of computer vision, a camera can see and identify a land mark and how far its position pixel-wise from the center of the picture frame, and import it to our script as a tuple, the aircraft will keep adjusting its vertical and horizontal speeds according to that tuple, until its completely centered or (0,0), thus the speeds.

The align action:
``` python
def align():

    flight_mode('GUIDED')
    K = 0.001
    done = False
    
    while not done:
        nav = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        speed_vector= np.sqrt(nav.vx**2 + nav.vy**2)
        Attitude = master.recv_match(type='AHRS2', blocking=True)
        heading, pitch, roll = Attitude.yaw, Attitude.pitch, Attitude.roll

        try:
            x, y = get_data()
            error_vector = np.sqrt(x**2 + y**2)
            error_relative_heading = np.arctan2(y, x)
            
        except:
            done = True
            return 'lost'
        
        else:

            compined_heading = heading + error_relative_heading
            step_x = np.cos(roll) * K * (error_vector * np.cos(compined_heading))
            step_y = np.cos(pitch) * K * (error_vector * np.sin(compined_heading))
            step_vector = np.sqrt(step_x**2 + step_y**2)

            if speed_vector != step_vector :
                set_vel_glob(step_x, step_y)

            if abs(speed_vector - step_vector) <= 0.1:
                done = True
                return 'aligned'
                
``` 
its important to align the opject relative heading to the drone with the heading of the drone itself, as the speeds are adjusted in the (local frame) of the drone
and ofcouse, inform the user if the aircraft is completely allinged or the reading of the opject position has lost.

DragKit are currently being devloped, new actions are being added!

Feel free to check DarkKit GitHub for the latest update, providing a feedback or push requests are welcome.
