# This code has been modified from the prepackaged controller for the Mantis class robot as follows:
#  1) Converted original controller from C code to Python
#  2) Added code to initialize lidar sensors, GPS unit, Compass unit and Display unit
#  3) Added code to dray map data onto the display attached to the Mantis' extension slot


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

# Initialize leg motors
motorNames = ['RPC', 'RPF', 'RPT', 'RMC', 'RMF', 'RMT', 'RAC', 'RAF', 'RAT', 'LPC', 'LPF', 'LPT', 'LMC', 'LMF', 'LMT', 'LAC', 'LAF', 'LAT']
motors = []
for i in range(0, 18):
    motors.append(robot.getDevice(motorNames[i]))
    
ds = robot.getDevice("Sharp's IR sensor GP2D120")
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
    
map = np.zeros(shape=(360,360))
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


# Main controller loop:
while robot.step(timestep) != -1:
    
    # Retrieve the pose coordinates from the GPS and Compass units
    time = robot.getTime()
    coord = gps.getValues()
    n = compass.getValues()
    pose_x = coord[0]
    pose_y = coord[2]
    pose_theta = -((math.atan2(n[0], n[2])) - (math.pi/2))

    # Nove the legs using a sinosoidal function
    for i in range(0, 18):  
        motors[i].setPosition(ampVector[i] * math.sin(2.0 * math.pi * frequency * time + phaseVector[i]) + offsetVector[i])
      
    lidarReadings = lidar.getRangeImage()
    #print(0.1594*ds.getValue()**(-.8533-0.2916))
    
    # Print the mapping data onto the display for debugging purposes.  Remove Display code/module for final demonstration
    for i, rho in enumerate(lidarReadings):
    
        # Compute rho and alpha from the sensor data
        alpha = lidarOffsetAngles[i-1]
        if rho > MAX_LIDAR_ANGLE:
            rho = MAX_LIDAR_ANGLE
            
        # Compute the robot/world coordinates for sensor data
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho
        wx =  (math.cos(pose_theta)*rx - math.sin(pose_theta)*ry) + pose_x
        wy =  (-math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y

        # Draw to the dispaly if the sensor output is within threshold
        if rho <= 0.54:
            (pixel_x, pixel_y) = (int(wx*4), 400-int(wy*4))
            
            # Ensure that the pixel to be drawn is within the bounds of the map
            if(pixel_x >= 0 and pixel_x < 360 and pixel_y >= 0 and pixel_y < 360):
                map[pixel_x][pixel_y] += 0.005
                
                # Filter out sensor noise
                if(map[pixel_x][pixel_y] >= 1.0):
                    map[pixel_x][pixel_y] = 1.0
                    
                # Print sensor data above threshold after filtering noise to the display
                g = int(map[pixel_x][pixel_y]*255)
                g = int((g*256**2+g*256+g))
                display.setColor(g)
                display.drawPixel(int(wx*4),400-int(wy*4))

    # Draw red pixels on robot location for debug purposes
    #display.setColor(int(0xFF0000))
    #display.drawPixel(int(pose_y*4),int(pose_x*4))
