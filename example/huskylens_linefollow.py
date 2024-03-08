import time
import board
import elio
from math import sqrt, acos, degrees

from circuitPyHuskyLib import HuskyLensLibrary

hl = HuskyLensLibrary('I2C', SCL=board.IO9, SDA=board.IO8)
hl.algorithm("ALGORITHM_LINE_TRACKING")
hl.clearText()

MOVE = True
ANGLE_THRESHOLD = 89
INIT_SPEED = 24

def Robot_Movement(action):
    if action == "FRONT":
        elio.moveForward(INIT_SPEED)
    elif action == "BACK":
        elio.moveBackward(INIT_SPEED)
    elif action == "LEFT":
        elio.turnLeft(INIT_SPEED)
    elif action == "RIGHT":
        elio.turnRight(INIT_SPEED)
    elif action == "END_LINE":
        elio.motorStop()

def find_line_midpoint(cor1, cor2):
    x1, y1 = cor1
    x2, y2 = cor2
    return (x1+x2)/2, (y1+y2)/2

def euclideanDist(p1, p2):
    x1,y1 = p1
    x2,y2 = p2
    return sqrt((x1-x2)**2+(y1-y2)**2)

def calibration(p1, p2):
    xmid, ymid = find_line_midpoint(p1, p2)
    if (xmid > 80 and xmid <= 240 and ymid > 60 and ymid <= 180):
        return None
    if (ymid > 0 and ymid <= 60):
        return "FRONT"
    if (ymid > 180 and ymid <= 240):
        return "BACK"
    if (xmid > 0 and xmid <= 80):
        return "LEFT"
    if (xmid > 240 and xmid <= 320):
        return "RIGHT"

def findPosition(p1, p2):
    x1,y1 = p1
    x2,y2 = p2
    x = x2-x1
    return True if x>0 else False

def findAngle(p1, p2):
    xmid, ymid = find_line_midpoint(p1, p2)
    x1, y1 = p1
    a_dist = euclideanDist((xmid, ymid), (xmid, y1))
    h_dist = euclideanDist((xmid, ymid), (x1, y1))
    o_dist = euclideanDist((x1, y1), (xmid, y1))
    if a_dist <= 0:
        a_dist = 1
    return degrees(acos((a_dist * a_dist + h_dist * h_dist - o_dist * o_dist) / (2.0 * a_dist * h_dist)))

def determine_action(deg, turn):
    if deg > ANGLE_THRESHOLD:
        return "END_LINE"
    elif deg < ANGLE_THRESHOLD / 2:
        return "FRONT"
    else:
        return "RIGHT" if turn else "LEFT"


while True:
    results = hl.learnedArrows()

    if MOVE and results:
        r = results[0]
        direction = calibration((r.xHead, r.yHead), (r.xTail, r.yTail))

        if direction:
            Robot_Movement(direction)
        else:
            turn = findPosition((r.xHead, r.yHead), (r.xTail, r.yTail))
            angle = findAngle((r.xHead, r.yHead), (r.xTail, r.yTail))
            action = determine_action(angle, turn)
            Robot_Movement(action)

    time.sleep(0.1)

