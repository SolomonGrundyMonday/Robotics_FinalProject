from controller import Robot, DistanceSensor, Motor
import math

# Definition of Base class for Youbot base controls
class Base:

    # Class constructor, initializes class constants and enables the wheel motors.
    def __init__(self, robot):
        self.motor_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
        self.wheels = []
        self.actualSpeed = [0.0, 0.0, 0.0]
        self.targetSpeed = [0.0, 0.0, 0.0]
        self.maxAcceleration = [10.0, 6.0, 20.0]
        
        self.WHEEL_RADIUS = 0.063
        self.DISTANCE_WHEEL_TO_ROBOT_CENTRE = 0.1826
        self.MAX_SPEED = 8
        self.DEMO_SPEED = 2
        self.OBSTACLE_THRESHOLD = 0.20
        
        for i in self.motor_names:
            self.wheels.append(robot.getDevice(i))
            
        for i in self.wheels:
            i.setPosition(float('inf'))
            i.setVelocity(0.0)
            
    # Definition for Base.base_reset function.  Included in original C controller (unused).
    def base_reset(self):
        for s in self.targetSpeed:
            s = 0.0
    
    # Definition for Base.base_forwards function.  Modified from version in original C controller.
    def base_forwards(self, velocity):
        if velocity > self.MAX_SPEED:
            velocity = self.MAX_SPEED
        
        for i in self.wheels:
            i.setVelocity(velocity)
    
    # Definition for Base.base_backwards function.  Modified from version in original C controller (unused).
    def base_backwards(self):
        for i in self.wheels:
            i.setVelocity(-self.MAX_SPEED)
    
    # Definition for Base.base_turn_left function.  Actually turns the base right - Modified from original C controller, normalizes input velocity.
    def base_turn_left(self, velocity):
        if(velocity > self.MAX_SPEED):
            velocity = self.MAX_SPEED
        
        self.wheels[1].setVelocity(velocity)
        self.wheels[3].setVelocity(velocity)
        self.wheels[0].setVelocity(-velocity)
        self.wheels[2].setVelocity(-velocity)
    
    # Definition for Base.base_turn_right function.  Actually turns the base left - Modified from original C controller, normalizes input velocity.                
    def base_turn_right(self, velocity):
        if(velocity > self.MAX_SPEED):
            velocity = self.MAX_SPEED
        
        self.wheels[0].setVelocity(velocity)
        self.wheels[2].setVelocity(velocity)
        self.wheels[1].setVelocity(-velocity)
        self.wheels[3].setVelocity(-velocity)
    
    # Definition for the Base.base_stop function.  Modified from original C controller.
    def base_stop(self):
        for i in self.wheels:
            i.setVelocity(0.0)