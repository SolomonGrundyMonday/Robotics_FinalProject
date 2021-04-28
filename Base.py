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
    
    
    def base_strafe_left(self):
        self.base_set_speeds(0, self.DEMO_SPEED / 2.0, 0)
    
    def base_strafe_right(self):
        self.base_set_speeds(0, -self.DEMO_SPEED / 2.0, 0)
    
    def base_apply_speeds(self, vx, vy, omega):
        vx /= self.WHEEL_RADIUS
        vy /= self.WHEEL_RADIUS
        omega *= self.DISTANCE_WHEEL_TO_ROBOT_CENTRE / self.WHEEL_RADIUS
        self.wheels[0].setVelocity(vy - omega)
        self.wheels[1].setVelocity(-math.sqrt(0.75) * vx - 0.5 * vy - omega)
        self.wheels[2].setVelocity(math.sqrt(0.75) * vx - 0.5 * vy - omega)
    
    def base_set_speeds(self, vx, vy, omega):
        self.targetSpeed[0] = vx;
        self.targetSpeed[1] = vy;
        self.targetSpeed[2] = omega;
    
    
    def base_braitenberg_avoidance(self, sensors_values):
    
        if (sensors_values[0] < self.OBSTACLE_THRESHOLD):
            self.base_backwards()
        elif (sensors_values[2] < self.OBSTACLE_THRESHOLD):
            self.base_strafe_right()
        elif (sensors_values[7] < self.OBSTACLE_THRESHOLD):
            self.base_strafe_left()
        if (sensors_values[4] < self.OBSTACLE_THRESHOLD or sensors_values[5] < OBSTACLE_THRESHOLD):
            self.base_forwards()
        elif (sensors_values[1] < self.OBSTACLE_THRESHOLD):
            self.base_turn_right()
        elif (sensors_values[8] < self.OBSTACLE_THRESHOLD):
            self.base_turn_left()
        elif (sensors_values[3] < self.OBSTACLE_THRESHOLD):
            self.base_turn_right()
        elif (sensors_values[6] < self.OBSTACLE_THRESHOLD):
            self.base_turn_left()
        else:
            self.base_forwards()