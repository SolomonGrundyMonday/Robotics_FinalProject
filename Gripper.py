from controller import Motor, Robot


class Gripper:

    def __init__(self, robot):
        self.MIN = 0.0   #doubt
        self.MAX = 0.025 #doubt
        self.OFFSET = 0.021
        
        print('Initialized constants!')
        
        self.fingers = []
        self.fingers.append(robot.getDevice('finger1'))
        self.fingers.append(robot.getDevice('finger2'))
        
        print('Initialized actuators!')
    
        for f in self.fingers:
            f.setVelocity(0.03)
            
        print('Set velocities!')
        
    def in_position(self):
        
        for i in self.fingers:
            
            if i.getVelocity() != 0.0:
                return False
        return True
    
    def grip(self):
        for f in self.fingers:
            f.setPosition(self.MIN)
    
    def release(self):
        for f in self.fingers:
            f.setPosition(self.MAX)
    
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