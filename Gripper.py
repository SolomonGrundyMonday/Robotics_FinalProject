from controller import Motor, Robot

# Class Gripper definition for gripping actuator of Youbot robot.
class Gripper:

    # Class constructor, initializes class constants and enables the finger motors.
    def __init__(self, robot):
        self.MIN = 0.0   
        self.MAX = 0.025 
        self.OFFSET = 0.021
        
        self.fingers = []
        self.fingers.append(robot.getDevice('finger1'))
        self.fingers.append(robot.getDevice('finger2'))
    
        for f in self.fingers:
            f.setVelocity(0.03)
    
    # Definition for Gripper.grip function.  Sets the finger position to min position - Modified from original C controller
    def grip(self):
        for f in self.fingers:
            f.setPosition(self.MIN)
    
    # Definition for Gripper.release function.  Sets the finger position to max position - Modified from original C controller.
    def release(self):
        for f in self.fingers:
            f.setPosition(self.MAX)
    
    # Definition for Gripper.release function.  From original C controller (unused).
    def set_gap(self,gap):
        v = 0.5 * (gap - self.OFFSET)
        a = self.MIN
        b = self.MAX
        if v > self.MIN:
            v = self.MIN
        else:
            if v < self.MAX:
                v = self.MAX
        for f in self.fingers:
            f.setPosition(v)