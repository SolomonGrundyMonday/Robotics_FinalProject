from controller import Robot, Supervisor, Field, Node
from Arm import *
from Gripper import *
from Base import *
import numpy as np

import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Initialize the base, arm and gripper of the youbot robot
base = Base(robot)
arm = Arm(robot)
gripper = Gripper(robot)

# Enable compass/gps modules
compass = robot.getDevice('compass')
compass.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)

# Initialize waypoints and state machine initial state
waypoints = [(22.26, 24.61), (22.2, 24.6), (22.03, 26.06), (26.0, 26.4), (28.7, 25.0)]#(25.5, 25.0), 
current_waypoint = waypoints.pop(0)
state = 'lower arm'

# Establish the gains for feedback controls
x_gain = 1.5
theta_gain = 2.0

# Define function to determine if we are at the end waypoint for feedback control navigation.
def is_endpoint(x, y):#(25.5, 25.0)
    if np.isclose(x, 28.7, 0.01) and np.isclose(y, 25.0, 0.01):
        return True
    else:
        return False
   
# Main entry point         
while (robot.step(timestep) != -1):

    # Get robot pose values
    coord = gps.getValues()
    bearing = compass.getValues()
    pose_x = coord[0]
    pose_y = coord[2]
    pose_theta = -math.atan2(bearing[0], bearing[2])+math.pi/2#-1.5708)
    
    # Initial state: robot moves into position to grab the repair materials
    if state == 'lower arm':
        
        gripper.release()
        arm.pick_up()
        
        if robot.getTime() > 3.0:
            state = 'grab'
            
    # Second state: robot grabs repair materials from the 'shelf'
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
            
    # Third state: robot lifts repair materials off of 'shelf'
    elif state == 'lift':
        
        arm.lift()
        if robot.getTime() > 43.8:
            state = 'drive'
            
    # Fourth state: robot drives the repair materials to desired location       
    elif state == 'drive':
        
        # Compute bearing and distance error
        bearing_error = pose_theta + math.atan2(current_waypoint[1] - pose_y, current_waypoint[0] - pose_x)
        dist_error = math.sqrt(math.pow(pose_x - current_waypoint[0], 2) + math.pow(pose_y - current_waypoint[1], 2))       
        if dist_error <= 0.1 and len(waypoints) != 0:
            current_waypoint = waypoints.pop(0)
         
        # Compute velocity for wheels   
        x_prime = dist_error * x_gain
        theta_prime = abs(bearing_error) * theta_gain       
        velocity = theta_prime + theta_gain
        
        # Feedback control
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
            
    # Fifth state: robot lines up with damaged section
    elif state == 'get_pos':
        goal_theta = 0.0
        
        if (pose_theta - goal_theta) > 0.04:
            base.base_turn_left(1.0)
            
        else:
            base.base_stop()
            current_waypoint = (28.78, 25.14)
            dist_error = math.sqrt(math.pow(pose_x - current_waypoint[0], 2) + math.pow(pose_y - current_waypoint[1], 2))
            
            if dist_error > 0.2:
                base.base_forwards(0.5)
            else:
                base.base_stop()
                state = 'put_down'
                
    # Sixth state: robot positions repair materials relative to the damaged section
    elif state == 'put_down':
        starttime = robot.getTime()
        
        arm.drop()
        if starttime > 163.488:
            state = 'finally'
            
    # Final state: Robot releases repair materials
    elif state == 'finally':
        gripper.release()
        