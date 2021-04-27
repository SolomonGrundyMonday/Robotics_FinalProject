from controller import Motor, Robot

OFFSET = 0.021

class Gripper:

    def __init__(self):
        self.MIN = 0.0   #doubt
        self.MAX = 0.025 #doubt
        self.fingers = []
        self.fingers.append(robot.getDevice('finger1'))
        self.fingers.append(robot.getDevice('finger2'))
    
        for f in fingers:
            self.f.setVelocity(0.03)
    
    def grip(self):
        for f in fingers:
            self.f.setPosition(MIN)
    
    def release(self):
        for f in fingers:
            self.f.setPosition(MAX)
    
    def set_gap(self,gap):
        v = 0.5 * (gap - OFFSET)
        a = MIN
        b = MAX
        if v > MIN:
            v = MIN
        else:
            if v < MAX:
                v = MAX
        for f in fingers:
            self.f.setPosition(v)
    
