"""This is not my work, it is the controller packaged with the mantis model robot transferred into python
   All credit for this controller goes to micromagic for this walking implementation"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motorNames = ['RPC', 'RPF', 'RPT', 'RMC', 'RMF', 'RMT', 'RAC', 'RAF', 'RAT', 'LPC', 'LPF', 'LPT', 'LMC', 'LMF', 'LMT', 'LAC', 'LAF', 'LAT']

motors = []

for i in range(0, 18):
    motors.append(robot.getDevice(motorNames[i]))
    
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
    
    time = robot.getTime();

    for i in range(0, 18):  
      motors[i].setPosition(ampVector[i] * math.sin(2.0 * math.pi * frequency * time + phaseVector[i]) + offsetVector[i]);

# Enter here exit cleanup code.
