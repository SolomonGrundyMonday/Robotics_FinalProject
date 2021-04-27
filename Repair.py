from controller import Keyboard, Robot
from Arm import *
from Gripper import *
from Base import *

import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

def passive_wait(sec):
    start_time = robot.getTime()
    while (start_time + sec > robot.getTime()):
        continue


def automatic_behavior():
    passive_wait(2.0)
    release()
    set_height(ARM_FRONT_CARDBOARD_BOX)
    passive_wait(4.0)
    grip()
    passive_wait(1.0)
    set_height(ARM_BACK_PLATE_LOW)
    passive_wait(3.0)
    release()
    passive_wait(1.0)
    arm_reset()
    passive_wait(5.0)
    grip()
    passive_wait(1.0)
    passive_wait(1.0)
    release()
    set_height(ARM_BACK_PLATE_LOW)
    passive_wait(3.0)
    grip()
    passive_wait(1.0)
    set_height(ARM_RESET)
    passive_wait(2.0);
    set_height(ARM_FRONT_PLATE)
    set_orientation(ARM_RIGHT)
    passive_wait(4.0)
    set_height(ARM_FRONT_FLOOR)
    passive_wait(2.0)
    release()
    passive_wait(1.0)
    set_height(ARM_FRONT_PLATE)
    passive_wait(2.0)
    set_height(ARM_RESET)
    passive_wait(2.0)
    arm_reset()
    grip()
    passive_wait(2.0)



arm_init(robot)
init(robot)
    
pc = 0

while (robot.step(timestep) != -1):

    c = keyboard.getKey()
    if ((c >= 0) and c != pc):
            # if c == WB_KEYBOARD_END:
                #do nothing
        if c == ' ':
            arm_reset()
        if c == 65585:
            gripper_grip()
        if c == 390:
            release()
        if c == Keyboard.UP or Keyboard.SHIFT:
            increase_height()
        if c == Keyboard.DOWN or Keyboard.SHIFT:
            decrease_height()
        if c == Keyboard.RIGHT or Keyboard.SHIFT:
            increase_orientation()
        if c == Keyboard.LEFT or Keyboard.SHIFT:
            decrease_orientation()
