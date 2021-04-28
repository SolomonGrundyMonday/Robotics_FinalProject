from controller import Keyboard, Robot, Supervisor, Field, Node
from Arm import *
from Gripper import *
from Base import *

import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

base = Base(robot)
arm = Arm(robot)
gripper = Gripper(robot)

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

gps = robot.getDevice('gps')
gps.enable(timestep)

waypoints = [(22.3, 24.7), (22.3, 26.2), (28.1, 26.4), (28.8, 25.3)]
current_waypoint = waypoints.pop(0)
state = 'lower arm'

while (robot.step(timestep) != -1):

    #arm.arm_reset()

    if state == 'lower arm':
        gripper.release()
        arm.pick_up()
        if gripper.in_position() and arm.in_position():
            state = 'grab'
    elif state == 'grab':
        gripper.grip()
        if gripper.in_position():
            state == 'lift'
    elif state == 'lift':
        arm.set_height(5)
        if arm.in_position():
            state = 'drive'       
    elif state == 'drive':
    
        coord = gps.getValues()
        bearing = compass.getValues()
        pose_x = coord[0]
        pose_y = coord[2]
        pose_theta = -((math.atan2(bearing[0], bearing[2]))-1.5708)
        
        bearing_error = pose_theta - math.atan2(current_waypoint[1] - pose_y, current_waypoint[0] - pose_x)
        dist_error = dist_err = math.sqrt(math.pow(pose_x - current_waypoint[0], 2) + math.pow(pose_y - current_waypoint[1], 2))
        
        if dist_error <= 0.1:
            current_waypoint = waypoints.pop(0)
            
        x_gain = 1.5
        theta_gain = 3.3
        x_prime = dist_error * x_gain
        theta_prime = bearing_error * theta_gain
        
        if(bearing_error > 0.5):
            vR = theta_prime + theta_gain
            vL = -theta_prime - theta_gain
            base.base_turn_right(vR, vL)
        elif(bearing_error < -0.5):
            vR = -theta_prime - theta_gain
            vL = theta_prime + theta_gain
            base.base_turn_left(vR, vL)
        elif (not waypoints and dist_error < 0.1):
            base.base_stop()
        else:
            v = x_prime + x_gain
            base.base_forwards(v)
    elif state == 'put down':
        continue
        
     
    
    #print('Current Waypoint: ', current_waypoint, ' Current Location: ', coord)
    #print('Bearing Error: ', bearing_error, ' DistanceError: ', dist_error)
    #c = keyboard.getKey()
    #if ((c >= 0)):
            
       #if c == Keyboard.END:
            #arm.arm_reset()
        #elif c == Keyboard.PAGEUP:
            #gripper.grip()
        #elif c == Keyboard.PAGEDOWN:
            #gripper.release()
        #elif c == Keyboard.UP + Keyboard.SHIFT:
            #arm.increase_height()
        #elif c == Keyboard.DOWN + Keyboard.SHIFT:
            #arm.decrease_height()
        #elif c == Keyboard.RIGHT + Keyboard.SHIFT:
            #arm.increase_orientation()
        #elif c == Keyboard.LEFT + Keyboard.SHIFT:
            #arm.decrease_orientation()
        #elif c == Keyboard.UP:
            #base.base_forwards()
        #elif c == Keyboard.DOWN:
            #base.base_backwards()
        #elif c == Keyboard.LEFT:
            #base.base_turn_left()
        #elif c== Keyboard.RIGHT:
            #base.base_turn_right()
        #elif c == Keyboard.HOME:
            #base.base_stop()
        #else:
            #arm.turn_wrist(-math.pi/2)
