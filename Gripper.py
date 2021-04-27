from controller import Motor, Robot

MIN = 0.0
MAX = 0.025
OFFSET = 0.021

fingers = []

def init(robot):
    fingers[0] = robot.getDevice('finger1')
    fingers[1] = robot.getDevice('finger2')

    for f in fingers:
        f.setVelocity(0.03)

def grip():
    for f in fingers:
        f.setPosition(MIN)

def release():
    for f in fingers:
        f.setPosition(MAX)

def set_gap(gap):
    v = 0.5 * (gap - OFFSET)
    a = MIN
    b = MAX
    if v > MIN:
        v = MIN
    else:
        if v < MAX:
            v = MAX
    for f in fingers:
        f.setPosition(v)
