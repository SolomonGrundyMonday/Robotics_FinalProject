from controller import Motor, Robot
import math

ARM_FRONT_FLOOR = 0
ARM_FRONT_PLATE = 1
ARM_HANOI_PREPARE = 2
ARM_FRONT_CARDBOARD_BOX = 3
ARM_RESET = 4
ARM_BACK_PLATE_HIGH = 5
ARM_BACK_PLATE_LOW = 6
ARM_MAX_HEIGHT = 7

ARM_BACK_LEFT = 0
ARM_LEFT = 1
ARM_FRONT_LEFT = 2
ARM_FRONT = 3
ARM_FRONT_RIGHT = 4
ARM_RIGHT = 5
ARM_BACK_RIGHT = 6
ARM_MAX_SIDE = 7


arms = []
current_height = 0
cureent_orientation = 0



def arm_init(robot):
    arms.append(robot.getDevice('arm1'))
    arms.append(robot.getDevice('arm2'))
    arms.append(robot.getDevice('arm3'))
    arms.append(robot.getDevice('arm4'))
    arms.append(robot.getDevice('arm5'))

    arms[2].setVelocity(0.5)

    set_height('reset')
    set_height('front')


def arm_reset():
    arm[0].setPosition(0.0)
    arm[1].setPosition(1.57)
    arm[2].setPosition(-2.635)
    arm[3].setPosition(1.78)
    arm[4].setPosition(0.0)


def set_height(value):

    if value == ARM_FRONT_FLOOR:
        arm[1].setPosition(-0.97)
        arm[2].setPosition(-1.55)
        arm[3].setPosition(-0.61)
        arm[4].setPosition(0.0)
    elif value == ARM_FRONT_PLATE:
        arm[1].setPosition(-0.62)
        arm[2].setPosition(-0.98)
        arm[3].setPosition(-1.53)
        arm[4].setPosition(0.0)
    elif value == ARM_FRONT_CARDBOARD_BOX:
        arm[1].setPosition(0.0)
        arm[2].setPosition(-0.77)
        arm[3].setPosition(-1.21)
        arm[4].setPosition(0.0)
    elif value == ARM_RESET:
        arm[1].setPosition(1.57)
        arm[2].setPosition(-2.635)
        arm[3].setPosition(1.78)
        arm[4].setPosition(0.0)
    elif value == ARM_BACK_PLATE_HIGH:
        arm[1].setPosition(0.678)
        arm[2].setPosition(0.682)
        arm[3].setPosition(1.74)
        arm[4].setPosition(0.0)
    elif value == ARM_BACK_PLATE_LOW:
        arm[1].setPosition(0.92)
        arm[2].setPosition(0.42)
        arm[3].setPosition(1.78)
        arm[4].setPosition(0.0)
    elif value == ARM_HANOI_PREPARE:
        arm[1].setPosition(-0.4)
        arm[2].setPosition(-1.2)
        arm[3].setPosition(-math.pi/2)
        arm[4].setPosition(math.pi/2)
    current_height = height;



def set_orientation(orientation):
    if orientation == ARM_BACK_LEFT:
        arm[0].setPosition(-2.949)
    elif orientation == ARM_LEFT:
        arm[0].setPosition(-math.pi/2)
    elif orientation == ARM_FRONT_LEFT:
        arm[0].setPosition(-0.2)
    elif orientation == ARM_FRONT:
        arm[0].setPosition(0.0)
    elif orientation == ARM_FRONT_RIGHT:
        arm[0].setPosition(0.2)
    elif orientation == ARM_RIGHT:
        arm[0].setPosition(math.pi/2)
    elif orientation == ARM_BACK_RIGHT:
        arm[0].setPosition(2.949)

def increase_height():
    current_height += 1
    if(current_height >= ARM_MAX_HEIGHT):
        current_height = ARM_MAX_HEIGHT - 1
    set_height(current_height)


def decrease_height():
    current_height -= 1
    if(current_height < 0)
        current_height = 0
    set_height(current_height)


def increase_orientation():
    current_orientation += 1
    if current_orientation >= ARM_MAX_SIDE:
        current_orientation = ARM_MAX_SIDE - 1
    set_orientation(current_orientation)


def decrease_orientation():
    current_orientation -= 1
    if current_orientation < 0:
        current_orientation = 0;
    set_orientation(current_orientation)

def set_sub_rotation(arm, rad):
    arm.setPosition(rad)


def get_sub_length(arm):
    l = 0.0
    if(arm == 0):
        l = 0.253
    elif(arm == 1):
        l = 0.155
    elif(arm == 2):
        l = 0.135
    elif(arm == 3):
        l = 0.081
    elif(arm = 4):
        l = 0.105
    return l


def inverse_kinematics(x, y, z):
    x1 = math.sqrt(x**2 + z**2)
    y1 = y + get_sub_length(3) + get_sub_length(4) + get_sub_length(0)

    a = get_sub_length(1)
    b = get_sub_length(2)
    c = math.sqrt(x1**2 + y1**2)

    alpha = -math.asin(z / x1)
    beta = -((math.pi/2) - math.acos((a**2 + c**2 - b**2) / (2.0 * a * c)) - math.atan(y1/x1))
    gamma = -(math.pi - math.acos((a**2 + b**2 - c**2) / (2.0 * a * b)))
    delta = -(math.pi + (beta + gamma))
    epsilon = (math.pi/2) + alpha

    arm[0].setPosition(alpha)
    arm[1].setPosition(beta)
    arm[2].setPosition(gamma)
    arm[3].setPosition(delta)
    arm[4].setPosition(epsilon)
