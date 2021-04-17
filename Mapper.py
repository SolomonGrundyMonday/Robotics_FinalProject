"""This is not my work, it is the controller packaged with the mantis model robot transferred into python
   All credit for this controller goes to micromagic for this walking implementation"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor
import math
import numpy as np

ANGLE_BINS = 667
MAX_LIDAR_ANGLE = math.radians(240)
LIDAR_SENSOR_MAX_RANGE = 5.5

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motorNames = ['RPC', 'RPF', 'RPT', 'RMC', 'RMF', 'RMT', 'RAC', 'RAF', 'RAT', 'LPC', 'LPF', 'LPT', 'LMC', 'LMF', 'LMT', 'LAC', 'LAF', 'LAT']

motors = []

for i in range(0, 18):
    motors.append(robot.getDevice(motorNames[i]))
    
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

display = robot.getDevice('display')

lidarReadings = []
lidarOffsetAngles = np.linspace(-MAX_LIDAR_ANGLE/2, MAX_LIDAR_ANGLE/2, ANGLE_BINS)
    
map = np.zeros(shape=(360,360))
frequency = 0.5

# amplitude [rad]
baseAmp = 0.25
shoulderAmp = 0.2
kneeAmp = 0.05
ampVector = [baseAmp, shoulderAmp, -kneeAmp, -baseAmp, -shoulderAmp, kneeAmp, baseAmp, shoulderAmp, -kneeAmp, baseAmp, -shoulderAmp, kneeAmp, -baseAmp, shoulderAmp, -kneeAmp, baseAmp, -shoulderAmp, kneeAmp]

# phase [s]
basePhase = 0.0
shoulderPhase = 2.0
kneePhase = 2.5
phaseVector = [basePhase, shoulderPhase, kneePhase, basePhase, shoulderPhase, kneePhase, basePhase, shoulderPhase, kneePhase, basePhase, shoulderPhase, kneePhase, basePhase, shoulderPhase, kneePhase, basePhase, shoulderPhase, kneePhase]

# offset [rad]
baseOffset = 0.6   
shoulderOffset = 0.8   
kneeOffset = -2.4  
offsetVector = [-baseOffset, shoulderOffset, kneeOffset, 0.0, shoulderOffset, kneeOffset, baseOffset, shoulderOffset, kneeOffset, baseOffset, shoulderOffset, kneeOffset, 0.0, shoulderOffset, kneeOffset, -baseOffset, shoulderOffset, kneeOffset]

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    time = robot.getTime()
    coord = gps.getValues()
    n = compass.getValues()
    pose_x = coord[0]
    pose_y = coord[2]
    pose_theta = -((math.atan2(n[0], n[2])) - (math.pi/2))
    
    #print('Coordinates: ', coord)

    for i in range(0, 18):  
        motors[i].setPosition(ampVector[i] * math.sin(2.0 * math.pi * frequency * time + phaseVector[i]) + offsetVector[i])
      
    lidarReadings = lidar.getRangeImage()
    
    for i, rho in enumerate(lidarReadings):
        alpha = lidarOffsetAngles[i-1]
        
        if rho > MAX_LIDAR_ANGLE:
            rho = MAX_LIDAR_ANGLE
            
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho
        
        wx =  (math.cos(pose_theta)*rx - math.sin(pose_theta)*ry) + pose_x
        wy =  (-math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y

        if rho < 1.0:

            (pixel_x, pixel_y) = (int(wx*3.6), int(wy*3.6))
            if(pixel_x >= 0 and pixel_x < 360 and pixel_y >= 0 and pixel_y < 360):
                map[pixel_x][pixel_y] += 0.005
                if(map[pixel_x][pixel_y] >= 1.0):
                    map[pixel_x][pixel_y] = 1.0
                g = int(map[pixel_x][pixel_y]*255)
                g = int((g*256**2+g*256+g))
                display.setColor(g)
                display.drawPixel(int(wx*3.66),int(wy*3.6))

    display.setColor(int(0xFF0000))
    display.drawPixel(int(pose_x*3.66),int(pose_y*3.66))

# Enter here exit cleanup code.