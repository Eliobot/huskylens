from elio import Motors, Buzzer, ObstacleSensor, LineSensor, WiFiConnectivity, IRRemote
import board
import time
import pwmio
import analogio
from math import sqrt
from circuitPyHuskyLib import HuskyLensLibrary

vBatt_pin = analogio.AnalogIn(board.BATTERY)

AIN1 = pwmio.PWMOut(board.IO36)
AIN2 = pwmio.PWMOut(board.IO38)
BIN1 = pwmio.PWMOut(board.IO35)
BIN2 = pwmio.PWMOut(board.IO37)

motors = Motors(AIN1, AIN2, BIN1, BIN2,vBatt_pin)

DIST_THRESHOLD = 30
SPEED = 0.02
MOVE = True

hl = HuskyLensLibrary('I2C', SCL=board.IO9, SDA=board.IO8)
hl.algorithm("ALGORITHM_FACE_RECOGNITION") # Redirect to object tracking


def euclideanDist(p1, p2=(160,120)):
    x1,y1 = p1
    x2,y2 = p2
    return sqrt((x1-x2)**2+(y1-y2)**2)

def findVerticalPos(p1, p2=(160,120)):
    _, y1 = p1
    _, y2 = p2
    y = y2-y1

    return euclideanDist((0,y1),(0,y2)) > DIST_THRESHOLD, 'FRONT' if y>0 else 'BACK'

def findHorizontalPos(p1, p2=(160,120)):
    x1, _ = p1
    x2, _ = p2
    x = x2-x1

    return euclideanDist((x1,0),(x2,0)) > DIST_THRESHOLD, 'LEFT' if x>0 else 'RIGHT'

def getVelocity(direction, speed=SPEED):
    VELOCITY = {
        'FRONT': lambda: motors.move_forward(int(speed * 100)+10),
        'BACK': lambda: motors.move_backward(int(speed * 100)+10),
        'LEFT': lambda: motors.turn_left(int(speed * 100)),
        'RIGHT': lambda: motors.turn_right(int(speed * 100)),
        'STOP': motors.motor_stop
    }

    return VELOCITY.get(direction, VELOCITY['STOP'])

def get_area(result):
    return result.ID, result.width * result.height

while True:

    results = hl.learnedBlocks() #
    # Only get learned results

    if MOVE and results:
        r = results[0]

        dist = euclideanDist((r.x,r.y))
        _, area = get_area(r)

        vThresh, vDirection = findVerticalPos((r.x,r.y))
        hThresh, hDirection = findHorizontalPos((r.x,r.y))

        if vThresh:
            action = getVelocity(vDirection)
        elif hThresh:
            action = getVelocity(hDirection)
        else:
            action = getVelocity('STOP')

        action() # Execute the motor action
    else:
        motors.motor_stop()

    time.sleep(0.1)

