import time

import cv2
import numpy as np

import vrep

# Close all the connections.
vrep.simxFinish(-1)
# Connect the V-REP.
clientID = vrep.simxStart("127.0.0.1", 19999, True, True, 5000, 5)

# Get object handles.
_, bot = vrep.simxGetObjectHandle(clientID, 'Car', vrep.simx_opmode_oneshot_wait)
_, vision_sensor = vrep.simxGetObjectHandle(clientID, 'Vision_Sensor', vrep.simx_opmode_oneshot_wait)

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
        edge = cv2.Canny(th2, 50, 150)  # type: np.ndarray
        return edge
    else:
        return None
