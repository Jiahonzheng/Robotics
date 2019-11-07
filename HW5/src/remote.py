import time

import cv2
import math
import numpy as np

import vrep

# Close all the connections.
vrep.simxFinish(-1)
# Connect the V-REP.
clientID = vrep.simxStart("127.0.0.1", 19999, True, True, 5000, 5)

# Get object handles.
_, bot = vrep.simxGetObjectHandle(clientID, 'MyBot', vrep.simx_opmode_oneshot_wait)
_, vision_sensor = vrep.simxGetObjectHandle(clientID, 'Vision_Sensor', vrep.simx_opmode_oneshot_wait)
_, left_motor = vrep.simxGetObjectHandle(clientID, 'Left_Motor', vrep.simx_opmode_oneshot_wait)
_, right_motor = vrep.simxGetObjectHandle(clientID, 'Right_Motor', vrep.simx_opmode_oneshot_wait)

if clientID == -1:
    raise Exception("Fail to connect remote API server.")


def init():
    """Initialize the simulation.
    """
    vrep.simxGetVisionSensorImage(clientID, vision_sensor, 0, vrep.simx_opmode_streaming)
    time.sleep(1)


def get_image(sensor):
    """Retrieve a binary image from Vision Sensor.

    :return: a binary image represented by numpy.ndarray from Vision Sensor
    """
    err, resolution, raw = vrep.simxGetVisionSensorImage(clientID, sensor, 0, vrep.simx_opmode_buffer)
    if err == vrep.simx_return_ok:
        img = np.array(raw, dtype=np.uint8)
        img.resize([resolution[1], resolution[0], 3])
        # Process the raw image.
        _, th1 = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)
        g = cv2.cvtColor(th1, cv2.COLOR_BGR2GRAY)
        _, th2 = cv2.threshold(g, 250, 255, cv2.THRESH_BINARY)
        # Find the edges using Canny.
        edge = cv2.Canny(th2, 50, 150)  # type: np.ndarray
        return edge
    else:
        return None


def move(v, o):
    """Move the robot.

    :param v: desired velocity
    :param o: desired angular velocity
    """
    wheel_radius = 0.027
    distance = 0.119
    v_l = v - o * distance
    v_r = v + o * distance
    o_l = v_l / wheel_radius
    o_r = v_r / wheel_radius
    vrep.simxSetJointTargetVelocity(clientID, left_motor, o_l, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, right_motor, o_r, vrep.simx_opmode_oneshot)


def get_beta_angle():
    """Return the degrees of Beta Euler Angle.

    :return: the degrees of Beta Euler Angle
    """
    _, euler_angles = vrep.simxGetObjectOrientation(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
    ret = math.degrees(euler_angles[1])
    if euler_angles[0] <= 0 < ret:
        return 180 - ret
    if euler_angles[2] <= 0 and ret < 0:
        return -180 - ret
    return ret
