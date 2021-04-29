from controller import Keyboard, Robot, Supervisor, Field, Node
from Arm import *
from Gripper import *
from Base import *
import numpy as np

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

waypoints = [(22.26, 24.61), (22.2, 24.6), (22.03, 26.06), (26.0, 26.4), (28.7, 25.0)]#(25.5, 25.0), 
current_waypoint = waypoints.pop(0)
state = 'lower arm'

x_gain = 1.5
theta_gain = 2.0

def is_endpoint(x, y):#(25.5, 25.0)
    if np.isclose(x, 28.7, 0.01) and np.isclose(y, 25.0, 0.01):
        return True
    else:
        return False
            
while (robot.step(timestep) != -1):

    #arm.arm_reset()
    #print(gps.getValues()[0], gps.getValues()[2])
    coord = gps.getValues()
    bearing = compass.getValues()
    pose_x = coord[0]
    pose_y = coord[2]
    pose_theta = -math.atan2(bearing[0], bearing[2])+math.pi/2#-1.5708)
    print ("pose_x, pose_y",pose_x, pose_y)
    if state == 'lower arm':
        
        gripper.release()
        arm.pick_up()
        #arm.set_height(5)
        
        #arm.inverse_kinematics(0.01, 0.1, 0.375)
        if robot.getTime() > 3.0:
            state = 'grab'
    elif state == 'grab':
        pose_x = gps.getValues()[0]
        pose_y = gps.getValues()[2]
        
        dist_error = math.sqrt(math.pow(pose_x - current_waypoint[0], 2) + math.pow(pose_y - current_waypoint[1], 2))
        base.base_forwards(dist_error * x_gain)
        
        if dist_error <= 0.01:
            base.base_stop()
            gripper.grip()
            current_waypoint = waypoints.pop(0)
            state = 'lift'
    elif state == 'lift':
        
        #arm.increase_height()
        
        arm.lift()
        if robot.getTime() > 43.8:
            state = 'drive'       
    elif state == 'drive':
    
        #print('current waypoint: ', current_waypoint, ' coordinates: ', pose_x, ', ', pose_y)
        
        
        bearing_error = pose_theta + math.atan2(current_waypoint[1] - pose_y, current_waypoint[0] - pose_x)
     
        #rad = -((math.atan2(n[0], n[2]))-1.5708)
        #bearing_err = pose_theta - math.atan2(current_waypoint[1] - pose_y, current_waypoint[0] - pose_x)
        #bearing_error = pose_theta - math.atan2(current_waypoint[1] - pose_y, current_waypoint[0] - pose_x)
        dist_error = math.sqrt(math.pow(pose_x - current_waypoint[0], 2) + math.pow(pose_y - current_waypoint[1], 2))
        
        if dist_error <= 0.1 and len(waypoints) != 0:
            current_waypoint = waypoints.pop(0)
            
        x_prime = dist_error * x_gain
        theta_prime = abs(bearing_error) * theta_gain
        
        velocity = theta_prime + theta_gain
        
        # print('current waypoint: ', current_waypoint)
        # print('current pose: (', pose_x, pose_y, pose_theta, )
        
        # print('bearing error: ', bearing_error)
        # print('bearing: ', bearing)
        
        if(bearing_error > 0.01):           
            base.base_turn_left(velocity)
        elif(bearing_error < -0.01):
            base.base_turn_right(velocity)
        elif (not waypoints and dist_error < 0.1 and is_endpoint(pose_x, pose_y)):
            state = 'get_pos'
            base.base_stop()
        else:
            v = x_prime + x_gain
            base.base_forwards(v)
    elif state == 'get_pos':
        goal_theta = 0.0
        # print ('pose_theta - goal_theta', pose_theta - goal_theta)
        if (pose_theta - goal_theta) > 0.04:
            base.base_turn_left(1.0)
            
        else:
            base.base_stop()
            current_waypoint = (28.78, 25.14)
            dist_error = math.sqrt(math.pow(pose_x - current_waypoint[0], 2) + math.pow(pose_y - current_waypoint[1], 2))
            print ('dist_error', dist_error)
            if dist_error > 0.2:
                base.base_forwards(0.5)
            else:
                base.base_stop()
                state = 'put_down'
    elif state == 'put_down':
        arm.drop()
        print ('nice')
        
     
    
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
