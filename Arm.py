from controller import Motor, Robot
import math


class Arm:
    
    def __init__(self, robot):
    
        self.ARM_FRONT_FLOOR = 0
        self.ARM_FRONT_PLATE = 1
        self.ARM_HANOI_PREPARE = 2
        self.ARM_FRONT_CARDBOARD_BOX = 3
        self.ARM_RESET = 4
        self.ARM_BACK_PLATE_HIGH = 5
        self.ARM_BACK_PLATE_LOW = 6
        self.ARM_MAX_HEIGHT = 7
    
        self.ARM_BACK_LEFT = 0
        self.ARM_LEFT = 1
        self.ARM_FRONT_LEFT = 2
        self.ARM_FRONT = 3
        self.ARM_FRONT_RIGHT = 4
        self.ARM_RIGHT = 5
        self.ARM_BACK_RIGHT = 6
        self.ARM_MAX_SIDE = 7
        
        self.arms = []
        self.arms.append(robot.getDevice('arm1'))
        self.arms.append(robot.getDevice('arm2'))
        self.arms.append(robot.getDevice('arm3'))
        self.arms.append(robot.getDevice('arm4'))
        self.arms.append(robot.getDevice('arm5'))
    
        self.arms[2].setVelocity(0.5)
    
        self.set_height(4)
        self.set_orientation(3)
        
        self.current_height = 0
        self.current_orientation = 0
        
    def drop(self):
        self.arms[0].setPosition(2.85)
        self.arms[1].setPosition(1.3)
        # self.arms[4].setPosition(math.pi/3)
        
    def pick_up(self):
        self.arms[0].setPosition(2.9496)
        self.arms[4].setPosition(math.pi/2)
        self.arms[2].setPosition(0.3)
        self.arms[3].setPosition(0.15)
        self.arms[1].setPosition(1.13)
        
    def lift(self):
        #self.arms[1].setPosition(0.678)
        self.arms[2].setPosition(0.25)
        self.arms[3].setPosition(0.08)
        self.arms[4].setPosition(math.pi/2)
    
    def arm_reset(self):
        self.arms[0].setPosition(0.0)
        self.arms[1].setPosition(1.57)
        self.arms[2].setPosition(-2.635)
        self.arms[3].setPosition(1.78)
        self.arms[4].setPosition(0.0)
    
    def in_position(self):
        for i in self.arms:
            
            if i.getVelocity() != 0.0:
                return False
        return True
    
    def set_height(self, value):
    
        if value == self.ARM_FRONT_FLOOR:
            self.arms[1].setPosition(-0.97)
            self.arms[2].setPosition(-1.55)
            self.arms[3].setPosition(-0.61)
            self.arms[4].setPosition(math.pi/2)
        elif value == self.ARM_FRONT_PLATE:
            self.arms[1].setPosition(-0.62)
            self.arms[2].setPosition(-0.98)
            self.arms[3].setPosition(-1.53)
            self.arms[4].setPosition(math.pi/2)
        elif value == self.ARM_FRONT_CARDBOARD_BOX:
            self.arms[1].setPosition(0.0)
            self.arms[2].setPosition(-0.77)
            self.arms[3].setPosition(-1.21)
            self.arms[4].setPosition(math.pi/2)
        elif value == self.ARM_RESET:
            self.arms[1].setPosition(1.57)
            self.arms[2].setPosition(-2.635)
            self.arms[3].setPosition(1.78)
            self.arms[4].setPosition(math.pi/2)
        elif value == self.ARM_BACK_PLATE_HIGH:
            self.arms[1].setPosition(0.678)
            self.arms[2].setPosition(0.682)
            self.arms[3].setPosition(1.74)
            self.arms[4].setPosition(math.pi/2)
        elif value == self.ARM_BACK_PLATE_LOW:
            self.arms[1].setPosition(0.92)
            self.arms[2].setPosition(0.42)
            self.arms[3].setPosition(1.78)
            self.arms[4].setPosition(math.pi/2)
        elif value == self.ARM_HANOI_PREPARE:
            self.arms[1].setPosition(-0.4)
            self.arms[2].setPosition(-1.2)
            self.arms[3].setPosition(-math.pi/2)
            self.arms[4].setPosition(math.pi/2)
        self.current_height = value
    
    
    
    def set_orientation(self, orientation):
        if orientation == self.ARM_BACK_LEFT:
            self.arms[0].setPosition(-2.949)
        elif orientation == self.ARM_LEFT:
            self.arms[0].setPosition(-math.pi/2)
        elif orientation == self.ARM_FRONT_LEFT:
            self.arms[0].setPosition(-0.2)
        elif orientation == self.ARM_FRONT:
            self.arms[0].setPosition(0.0)
        elif orientation == self.ARM_FRONT_RIGHT:
            self.arms[0].setPosition(0.2)
        elif orientation == self.ARM_RIGHT:
            self.arms[0].setPosition(math.pi/2)
        elif orientation == self.ARM_BACK_RIGHT:
            self.arms[0].setPosition(2.949)
    
    def increase_height(self):
        self.current_height += 1
        if(self.current_height >= self.ARM_MAX_HEIGHT):
            self.current_height = self.ARM_MAX_HEIGHT - 1
        self.set_height(self.current_height)
    
    
    def decrease_height(self):
        self.current_height -= 1
        if(self.current_height < 0):
            self.current_height = 0
        self.set_height(self.current_height)
    
    
    def increase_orientation(self):
        self.current_orientation += 1
        if self.current_orientation >= self.ARM_MAX_SIDE:
            self.current_orientation = self.ARM_MAX_SIDE - 1
        self.set_orientation(self.current_orientation)
    
    
    def decrease_orientation(self):
        self.current_orientation -= 1
        if self.current_orientation < 0:
            self.current_orientation = 0;
        self.set_orientation(self.current_orientation)
    
    def set_sub_rotation(self, arm, rad):
        self.arm.setPosition(rad)
    
    
    def get_sub_length(self, arm):
        l = 0.0
        if(arm == 0):
            l = 0.253
        elif(arm == 1):
            l = 0.155
        elif(arm == 2):
            l = 0.135
        elif(arm == 3):
            l = 0.081
        elif(arm == 4):
            l = 0.105
        return l
    
    def turn_wrist(self, rad):
        self.arms[4].setPosition(rad)
        
    def get_wrist_pos(self):
        return self.arms[4].getPositionSensor().getValue()
    
    def inverse_kinematics(self, x, y, z):
        x1 = math.sqrt(x**2 + z**2)
        y1 = y + self.get_sub_length(3) + self.get_sub_length(4) + self.get_sub_length(0)
    
        a = self.get_sub_length(1)
        b = self.get_sub_length(2)
        c = math.sqrt(x1**2 + y1**2)
    
    
        numerator = a**2 + c**2 - b**2
        denominator = 2.0 * a * c
        print('a: ', a, 'b: ', b, 'c: ', c)
        print('x1: ', x1, 'y1: ', y1)
        print('a**2 + c**2 - b**2: ', numerator)
        print('2.0 * a * c: ', denominator)
        print('x1: ', x1, ' y1: ', y1)
        alpha = -math.asin(z / x1)
        beta = -((math.pi/2) - math.acos((a * a + c * c - b * b) / (2.0 * a * c)) - math.atan(y1/x1))
        gamma = -(math.pi - math.acos((a**2 + b**2 - c**2) / (2.0 * a * b)))
        delta = -(math.pi + (beta + gamma))
        epsilon = (math.pi/2) + alpha
    
        self.arms[0].setPosition(alpha)
        self.arms[1].setPosition(beta)
        self.arms[2].setPosition(gamma)
        self.arms[3].setPosition(delta)
        self.arms[4].setPosition(epsilon)
    
