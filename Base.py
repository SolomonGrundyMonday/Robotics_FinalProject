from controller import Robot, DistanceSensor, Motor
import math

class Base:
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
            
    
    def base_reset(self):
        for s in self.targetSpeed:
            s = 0.0
    
    def base_forwards(self):
        for i in self.wheels:
            i.setVelocity(self.MAX_SPEED)
    
    
    def base_backwards(self):
        for i in self.wheels:
            i.setVelocity(-self.MAX_SPEED)
    
    def base_turn_left(self):
        for i in range(len(self.wheels)):
            if i % 2 == 0:
                self.wheels[i].setVelocity(-self.MAX_SPEED)
            else:
                self.wheels[i].setVelocity(self.MAX_SPEED)
                    
    def base_turn_right(self):
        for i in range(len(self.wheels)):
            if i % 2 == 1:
                self.wheels[i].setVelocity(-self.MAX_SPEED)
            else:
                self.wheels[i].setVelocity(self.MAX_SPEED)
    
    def base_stop(self):
        for i in self.wheels:
            i.setVelocity(0.0)