import time

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import random
import traceback
import cv2
import ArucoDetectionSim

# Connect to the CoppeliaSim Remote API
client = RemoteAPIClient()
sim = client.require('sim')
sim.loadScene('/Users/mavri/Documents/Механика/АОвП/first_try.ttt')

# Run a simulation in stepping mode
# Global variables
vehicleHandle = None
cameraHandle = None
cat1 = None
cat2 = None
sensor_front = None
sound = None
sound2 = None
sensor_left = None
sensor_right = None
diff = None
side = None
currentHeading = None

# Установка светлого фона сцены
background_color = [1, 1, 1]  # Белый цвет (R, G, B)

leftDynamicMotors = [0, 0, 0, 0]
rightDynamicMotors = [0, 0, 0, 0]

leftVelocity = 0.0
rightVelocity = 0.0
acceleration = 0.0
leftTargetVelocity = 0.0
rightTargetVelocity = 0.0

dist_fr = 100.0
dist_fr2 = 100.0
count_sides = 0
count_sides_turn = 0

targetHeading = 0

state = 'done'
turning = 'turn'
course = 'nice'

wheelRadius = 0.0704
interTrackDistance = 0.24
dynamicSimulation = True


# Initialize the simulation
def sysCall_init():
    global vehicleHandle, cat1, cat2, background_color
    global sensor_front, sound, sound2, sensor_left, sensor_right, cameraHandle
    vehicleHandle = sim.getObject('./caterpillar')
    cat1 = sim.getObject('./caterpillar/cat1')
    cat2 = sim.getObject('./caterpillar/cat2')
    cameraHandle = sim.getObject('./camera')
    # sim.setVisionSensorImage(cameraHandle, background_color, 0)

    for i in range(4):
        leftDynamicMotors[i] = sim.getObject(f'./dynamicLeftJoint{i + 1}')
        rightDynamicMotors[i] = sim.getObject(f'./dynamicRightJoint{i + 1}')

    sound = sim.getObject('./sound')
    sound2 = sim.getObject('./sound2')

    sensor_left = sim.getObject('./sensorL')
    sensor_right = sim.getObject('./sensorR')


# Function to capture an image from the camera and detect Aruco markers
# def capture():
#     global cameraHandle, cameraMatrix, distCoeffs
#     # Get the image from the camera
#     img, resX, resY = sim.getVisionSensorCharImage(cameraHandle)
#     img = np.frombuffer(img, np.uint8).reshape((resY, resX, 3))
#     img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), 0)
#
#     # Convert the image to grayscale
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     return gray

# def capture():
#     global cameraHandle
#     image = sim.getVisionSensorImage(cameraHandle)
#
#     image = np.asarray(image).astype(np.uint8)
#
#     image = np.flip(image, 0)
#     image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
#     image = image.reshape([resolution[1], resolution[0], 3])
#     cv2.imwrite('base.png', image)
#     time.sleep(40)
#     print('done')
#

# Function to capture an image from the camera and detect Aruco markers
def capture():
    global cameraHandle, cameraMatrix, distCoeffs
    # Get the image from the camera
    img, resX, resY = sim.getVisionSensorCharImage(cameraHandle)
    img = np.frombuffer(img, dtype=np.uint8).reshape((resY, resX, 3))
    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), 0)

    # Convert the image to grayscale
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite('baza.png', img)
    return img


# Function to calculate the difference between two headings
def headingDiff(a, b):
    res = (a - b) % (2 * math.pi)
    if res > math.pi:
        return 2 * math.pi - res
    else:
        return res


# Function to turn the robot towards the target heading
def turnToHeading():
    global targetHeading, state, turning, course, wheelRadius, interTrackDistance, dynamicSimulation, currentHeading, diff
    global leftVelocity, rightVelocity, acceleration, leftTargetVelocity, rightTargetVelocity
    currentHeading = sim.getObjectOrientation(vehicleHandle, sim.handle_parent)[2]
    diff = headingDiff(currentHeading, targetHeading)
    if math.degrees(targetHeading) < 0:
        leftTargetVelocity = -2
        rightTargetVelocity = 2
    else:
        leftTargetVelocity = 2
        rightTargetVelocity = -2

    if math.degrees(abs(diff)) > 2:
        acceleration = diff * (-0.01)

    else:
        state = 'done'
        leftVelocity = 0.03
        rightVelocity = 0.03
        leftTargetVelocity = 0.03
        rightTargetVelocity = 0.03
        if abs(targetHeading) < math.radians(5):
            targetHeading = math.radians(0)


# Main logic function
def go():
    global dist_fr, dist_fr2, count_sides, count_sides_turn, side
    global course, turning, state, targetHeading
    if (dist_fr != 0.0 and dist_fr <= 0.3) and (
            dist_fr2 != 0.0 and dist_fr2 <= 0.3) or state == 'progress' or course != 'nice':
        if course != 'turning':
            side = random.randint(1, 2)
        if side == 2:
            turn_left()
        else:
            turn_right()
    else:
        targetHeading = math.radians(0)
        turning = 'turn'
        count_sides = 0


def turn_left():
    global targetHeading, state, turning, course, wheelRadius, interTrackDistance, dynamicSimulation, currentHeading, diff
    global leftVelocity, rightVelocity, acceleration, leftTargetVelocity, rightTargetVelocity
    global dist_fr, dist_fr2, count_sides, count_sides_turn, currentHeading
    if targetHeading == math.radians(0) and state == 'done' and turning == 'turn':
        targetHeading = math.radians(90)
        state = 'progress'
        turning = 'turn2'
        course = 'turning'
        count_sides_turn += 1
    if targetHeading == math.radians(90) and state == 'done' and turning == 'turn2':
        course = 'turning'
        if count_sides == 3:
            turning = 'turn2_1'
            count_sides = 0
        count_sides += 1
    if targetHeading == math.radians(90) and state == 'done' and turning == 'turn2_1':
        course = 'turning'
        if count_sides == 3:
            targetHeading = math.radians(-1)
            state = 'progress'
            turning = 'turn3'
            count_sides = 0
        count_sides += 1
    if targetHeading == math.radians(0) and state == 'done' and turning == 'turn3':
        course = 'turning'
        if count_sides == 3:
            turning = 'turn3_1'
            count_sides = 0
        count_sides += 1
    if targetHeading == math.radians(0) and state == 'done' and turning == 'turn3_1':
        course = 'turning'
        if count_sides == 3:
            targetHeading = math.radians(-90)
            state = 'progress'
            turning = 'turn4'
            count_sides = 0
            count_sides_turn = 0
        count_sides += 1
    if targetHeading == math.radians(-90) and state == 'done' and turning == 'turn4':
        targetHeading = math.radians(4)
        state = 'progress'
        turning = 'done'
        course = 'nice'
        count_sides = 0


def turn_right():
    global targetHeading, state, turning, course, count_sides
    if targetHeading == math.radians(0) and state == 'done' and turning == 'turn':
        targetHeading = math.radians(-90)


# Actuation function
def sysCall_actuation():
    global leftVelocity, rightVelocity, leftTargetVelocity, rightTargetVelocity, acceleration, targetHeading, state, turning, currentHeading
    dt = sim.getSimulationTimeStep()

    turnToHeading()
    # go()
    detect, x, z, y, mark = ArucoDetectionSim.ArucoDetector(capture())
    print(detect, x, z, y, mark)
    print(targetHeading, currentHeading)
    if detect:
        if mark == 20:
            targetHeading = math.radians(90)
        if mark == 15:
            targetHeading = math.radians(-5)

    if leftTargetVelocity > leftVelocity:
        leftVelocity += acceleration * dt
        if leftVelocity > leftTargetVelocity:
            leftVelocity = leftTargetVelocity
    else:
        leftVelocity -= acceleration * dt
        if leftVelocity < leftTargetVelocity:
            leftVelocity = leftTargetVelocity

    if rightTargetVelocity > rightVelocity:
        rightVelocity += acceleration * dt
        if rightVelocity > rightTargetVelocity:
            rightVelocity = rightTargetVelocity
    else:
        rightVelocity -= acceleration * dt
        if rightVelocity < rightTargetVelocity:
            rightVelocity = rightTargetVelocity

    sim.writeCustomTableData(cat1, '__ctrl__', {'vel': rightVelocity})
    sim.writeCustomTableData(cat2, '__ctrl__', {'vel': leftVelocity})

    if dynamicSimulation:
        for i in range(4):
            sim.setJointTargetVelocity(leftDynamicMotors[i], -leftVelocity / wheelRadius)
            sim.setJointTargetVelocity(rightDynamicMotors[i], -rightVelocity / wheelRadius)

        d = interTrackDistance
        linVar = sim.getSimulationTimeStep() * (leftVelocity + rightVelocity) / 2.0
        rotVar = sim.getSimulationTimeStep() * math.atan((rightVelocity - leftVelocity) / d)
        position = sim.getObjectPosition(vehicleHandle, sim.handle_parent)
        orientation = sim.getObjectOrientation(vehicleHandle, sim.handle_parent)
        xDir = [math.cos(orientation[2]), math.sin(orientation[2]), 0.0]
        position[0] += xDir[0] * linVar
        position[1] += xDir[1] * linVar
        orientation[2] += rotVar
        sim.setObjectPosition(vehicleHandle, sim.handle_parent, position)
        sim.setObjectOrientation(vehicleHandle, sim.handle_parent, orientation)


# Sensing function
def sysCall_sensing():
    global dist_fr, dist_fr2
    res_fr, dist_fr, pt_fr, x, y = sim.readProximitySensor(sound)
    res_fr2, dist_fr2, pt_fr2, x, y = sim.readProximitySensor(sound2)
    res_l, dist_l, pt_l, x, y = sim.handleProximitySensor(sensor_left)
    res_r, dist_r, pt_r, x, y = sim.handleProximitySensor(sensor_right)


try:
    sim.startSimulation()
    sysCall_init()
    while True:
        sysCall_actuation()
        sysCall_sensing()
except Exception as err:
    traceback.format_exc(traceback.format_exc(Exception, err))
    sim.stopSimulation()
