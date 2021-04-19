# This code has been modified from the prepackaged controller for the Mantis class robot as follows:
#  1) Converted original controller from C code to Python
#  2) Added code to initialize lidar sensors, GPS unit, Compass unit and Display unit
#  3) Added code to dray map data onto the display attached to the Mantis' extension slot


from controller import Robot, Motor, DistanceSensor
import math
import numpy as np

# Global constants for mapping code
ANGLE_BINS = 667
MAX_LIDAR_ANGLE = math.radians(240)
LIDAR_SENSOR_MAX_RANGE = 5.5
MAP_HEIGHT = 400
MAP_WIDTH = 400
DAMAGE_THRESHOLD = .74
INCREMENT_CMAP = 0.0005
NOISE_THRESHOLD = 1.0

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Initialize leg motors
motorNames = ['RPC', 'RPF', 'RPT', 'RMC', 'RMF', 'RMT', 'RAC', 'RAF', 'RAT', 'LPC', 'LPF', 'LPT', 'LMC', 'LMF', 'LMT', 'LAC', 'LAF', 'LAT']
motors = []
for i in range(0, 18):
    motors.append(robot.getDevice(motorNames[i]))
    
# ds = robot.getDevice("Delphi ESR")
ds = robot.getDevice("Sharp's IR sensor GP2Y0A710K0F")
ds.enable(timestep)
  
# Initialize lidar sensor  
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Initialize GPS unit
gps = robot.getDevice('gps')
gps.enable(timestep)

# Initialize Compass unit
compass = robot.getDevice('compass')
compass.enable(timestep)

# Initialize display and arrays for sensor readings and lidar beam angles
display = robot.getDevice('display')
lidarReadings = []
lidarOffsetAngles = np.linspace(-MAX_LIDAR_ANGLE/2, MAX_LIDAR_ANGLE/2, ANGLE_BINS)
    
map = np.zeros(shape=(MAP_WIDTH, MAP_HEIGHT))
frequency = 0.5

# amplitude (radians)
baseAmp = 0.25
shoulderAmp = 0.2
kneeAmp = 0.05
ampVector = [baseAmp, shoulderAmp, -kneeAmp, -baseAmp, -shoulderAmp, kneeAmp, baseAmp, shoulderAmp, -kneeAmp, baseAmp, -shoulderAmp, kneeAmp, -baseAmp, shoulderAmp, -kneeAmp, baseAmp, -shoulderAmp, kneeAmp]

# phase (seconds)
basePhase = 0.0
shoulderPhase = 2.0
kneePhase = 2.5
phaseVector = [basePhase, shoulderPhase, kneePhase, basePhase, shoulderPhase, kneePhase, basePhase, shoulderPhase, kneePhase, basePhase, shoulderPhase, kneePhase, basePhase, shoulderPhase, kneePhase, basePhase, shoulderPhase, kneePhase]

# offset (radians)
baseOffset = 0.6   
shoulderOffset = 0.8   
kneeOffset = -2.4  
offsetVector = [-baseOffset, shoulderOffset, kneeOffset, 0.0, shoulderOffset, kneeOffset, baseOffset, shoulderOffset, kneeOffset, baseOffset, shoulderOffset, kneeOffset, 0.0, shoulderOffset, kneeOffset, -baseOffset, shoulderOffset, kneeOffset]
goal_theta = 0

# Main controller loop:
while robot.step(timestep) != -1:
    
    # Retrieve the pose coordinates from the GPS and Compass units
    time = robot.getTime()
    coord = gps.getValues()
    n = compass.getValues()
    pose_x = coord[0]
    pose_y = coord[2]
    pose_theta = -((math.atan2(n[0], n[2])) - (math.pi/2))

    # print('x: ', pose_x, 'y: ', coord[1], 'z: ', pose_y)
    # Nove the legs using a sinusoidal function
    # 1.962*x^(-0.5214)+0.4926 
    # y[m] = f(x[V])  => y = 20.24*x^(-4.76)+0.6632
    distance = 20.24*pow(ds.getValue(),-4.76) +0.6632 #0.1594 * pow(ds.getNumberOfTargets(), -0.8533) -0.02916
    # distance = 
    bearing_err = pose_theta - goal_theta
    print ("distance: ", distance)
    print ("pose theta: ", pose_theta, "goal theta: ", goal_theta)
    print ("bearing_err: ", bearing_err)
    # if distance < 5:# if we hit the wall, start make it turn
    
        
    if abs(bearing_err) > 0.1 and pose_y < 5: #south        
        goal_theta = math.pi     
        for i in range(0, 9):#all the right legs
            motors[i].setPosition(ampVector[i] * math.sin(2.0 * math.pi * frequency * time + phaseVector[i]) + offsetVector[i])
        for i in range(9, 18):
            motors[i].setPosition(ampVector[i] * math.sin(2.0 * math.pi * frequency * time - phaseVector[i]) + offsetVector[i])
    elif abs(bearing_err) > 0.1 and pose_y > 95:
    
        goal_theta = 0
        for i in range(0, 9):#all the right legs
            motors[i].setPosition(ampVector[i] * math.sin(2.0 * math.pi * frequency * time - phaseVector[i]) + offsetVector[i])
        for i in range(9, 18):#all the left legs
            motors[i].setPosition(ampVector[i] * math.sin(2.0 * math.pi * frequency * time + phaseVector[i]) + offsetVector[i])
        
    # else:#facing north
        # for i in range(0, 9):#all the right legs
            # motors[i].setPosition(ampVector[i] * math.sin(2.0 * math.pi * frequency * time + phaseVector[i]) + offsetVector[i])
               
    else:#go straight forward
        for i in range(0, 18):  
            motors[i].setPosition(ampVector[i] * math.sin(2.0 * math.pi * frequency * time + phaseVector[i]) + offsetVector[i])
      
    lidarReadings = lidar.getRangeImage()
    
    # Print the mapping data onto the display for debugging purposes.  Remove Display code/module for final demonstration
    for i, rho in enumerate(lidarReadings):
        # Compute rho and alpha from the sensor data
        alpha = lidarOffsetAngles[i]
        if rho > LIDAR_SENSOR_MAX_RANGE:
            rho = LIDAR_SENSOR_MAX_RANGE
            
        # Compute the robot/world coordinates for sensor data
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho
        wx = (math.cos(pose_theta)*rx - math.sin(pose_theta)*ry) + pose_x
        wy = -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y

        # Draw to the display if the sensor output is within threshold
        if rho > DAMAGE_THRESHOLD and alpha <= (math.pi/4) and alpha >= -(math.pi/4):
            (pixel_x, pixel_y) = (400-int(wx*4), 400-int(wy*4))
            
            # Ensure that the pixel to be drawn is within the bounds of the map
            if(pixel_x >= 0 and pixel_x < MAP_WIDTH and pixel_y >= 0 and pixel_y < MAP_HEIGHT):
                map[pixel_x][pixel_y] += INCREMENT_CMAP
                
                # Filter out sensor noise
                if(map[pixel_x][pixel_y] > NOISE_THRESHOLD):
                    map[pixel_x][pixel_y] = NOISE_THRESHOLD
                    
                # Print sensor data above threshold after filtering noise to the display
                g = int(map[pixel_x][pixel_y]*255)
                g = int((g*256**2+g*256+g))
                display.setColor(g)
                display.drawPixel(pixel_x, pixel_y)
                
