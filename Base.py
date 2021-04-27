from controller import Robot, DistanceSensor, Motor
import math

WHEEL_RADIUS = 0.063
DISTANCE_WHEEL_TO_ROBOT_CENTRE = 0.1826
MAX_SPEED = 8
DEMO_SPEED = 2
OBSTACLE_THRESHOLD = 0.20

motor_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
wheels = []

actualSpeed = [0.0, 0.0, 0.0]
targetSpeed = [0.0, 0.0, 0.0]
maxAcceleration = [10.0, 6.0, 20.0]

def base_init(robot):
    for i in motor_names:
        wheels.append(robot.getDevice(i))
        

def base_reset():
    for s in targetSpeed:
        s = 0.0

def base_forwards():
    base_set_speeds(DEMO_SPEED / 2.0, 0, 0)


def base_backwards():
    base_set_speeds(-DEMO_SPEED / 2.0, 0, 0)

def base_turn_left():
    base_set_speeds(0, 0, 3.0 * DEMO_SPEED / 2.0)


def base_turn_right():
    base_set_speeds(0, 0, -3.0 * DEMO_SPEED / 2.0)


def base_strafe_left():
    base_set_speeds(0, DEMO_SPEED / 2.0, 0)

def base_strafe_right():
    base_set_speeds(0, -DEMO_SPEED / 2.0, 0)

def base_apply_speeds(vx, vy, omega):
    vx /= WHEEL_RADIUS
    vy /= WHEEL_RADIUS
    omega *= DISTANCE_WHEEL_TO_ROBOT_CENTRE / WHEEL_RADIUS
    wheels[0].setVelocity(vy - omega)

    wheels[1].setVelocity(-math.sqrt(0.75) * vx - 0.5 * vy - omega)
    wheels[2].setVelocity(sqrt(0.75) * vx - 0.5 * vy - omega)

def base_set_speeds(vx, vy, omega):
    targetSpeed[0] = vx;
    targetSpeed[1] = vy;
    targetSpeed[2] = omega;

def base_braitenberg_avoidance(sensors_values):

    if (sensors_values[0] < OBSTACLE_THRESHOLD):
        base_backwards()
    elif (sensors_values[2] < OBSTACLE_THRESHOLD):
        base_strafe_right()
    elif (sensors_values[7] < OBSTACLE_THRESHOLD):
        base_strafe_left()
    if (sensors_values[4] < OBSTACLE_THRESHOLD or sensors_values[5] < OBSTACLE_THRESHOLD):
        base_forwards()
    elif (sensors_values[1] < OBSTACLE_THRESHOLD):
        base_turn_right()
    elif (sensors_values[8] < OBSTACLE_THRESHOLD):
        base_turn_left()
    elif (sensors_values[3] < OBSTACLE_THRESHOLD):
        base_turn_right()
    elif (sensors_values[6] < OBSTACLE_THRESHOLD):
        base_turn_left()
    else:
        base_forwards()
