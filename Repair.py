from controller import Keyboard, Robot, Supervisor, Field, Node
from Arm import *
from Gripper import *
from Base import *

import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

print('initializing base')
base = Base(robot)
print('initializing arm')
arm = Arm(robot)
print('initializing gripper')
gripper = Gripper(robot)

print('Initialized all components!')
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

    
print('Emtering main control loop!')

while (robot.step(timestep) != -1):

    c = keyboard.getKey()
    if ((c >= 0)):
            # if c == WB_KEYBOARD_END:
                #do nothing
        if c == Keyboard.END:
            arm.arm_reset()
        elif c == Keyboard.PAGEUP:
            gripper.gripper_grip()
        elif c == Keyboard.PAGEDOWN:
            gripper.release()
        elif c == Keyboard.UP + Keyboard.SHIFT:
            arm.increase_height()
        elif c == Keyboard.DOWN + Keyboard.SHIFT:
            arm.decrease_height()
        elif c == Keyboard.RIGHT + Keyboard.SHIFT:
            arm.increase_orientation()
        elif c == Keyboard.LEFT + Keyboard.SHIFT:
            arm.decrease_orientation()
        elif c == Keyboard.UP:
            base.base_forwards()
        elif c == Keyboard.DOWN:
            base.base_backwards()
        elif c == Keyboard.LEFT:
            base.base_turn_left()
        elif c== Keyboard.RIGHT:
            base.base_turn_right()
        elif c == Keyboard.HOME:
            base.base_stop()
