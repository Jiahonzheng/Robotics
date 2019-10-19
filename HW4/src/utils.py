import time

import cv2
import imutils
import math
import numpy as np

import vrep

# Close all the connections.
vrep.simxFinish(-1)
# Connect the V-REP
clientID = vrep.simxStart("127.0.0.1", 19999, True, True, 5000, 5)

if clientID == -1:
    raise Exception("Fail to connect remote API server.")

_, l_motor = vrep.simxGetObjectHandle(clientID, 'MyBot_FLwheel_Motor', vrep.simx_opmode_oneshot_wait)
_, l_steer = vrep.simxGetObjectHandle(clientID, 'MyBot_FLwheel_Steer', vrep.simx_opmode_oneshot_wait)
_, r_motor = vrep.simxGetObjectHandle(clientID, 'MyBot_FRwheel_Motor', vrep.simx_opmode_oneshot_wait)
_, r_steer = vrep.simxGetObjectHandle(clientID, 'MyBot_FRwheel_Steer', vrep.simx_opmode_oneshot_wait)
_, lane_sensor = vrep.simxGetObjectHandle(clientID, 'MyBot_Lane_Vision_Sensor', vrep.simx_opmode_oneshot_wait)
_, obstacle_sensor = vrep.simxGetObjectHandle(clientID, 'MyBot_Obstacle_Vision_Sensor', vrep.simx_opmode_oneshot_wait)
_, proximity_sensor = vrep.simxGetObjectHandle(clientID, 'MyBot_Proximity_Sensor', vrep.simx_opmode_oneshot_wait)

# Display processed image, just for debugging!
_, debug = vrep.simxGetObjectHandle(clientID, 'Debug_Display', vrep.simx_opmode_oneshot_wait)

# The distance between left wheels and right wheels, 28 is the truth distance(cm), 0.2 is a factor.
distance_left_right = 28 * 0.2
# The distance between front wheels and rear wheels, 30 is the truth distance(cm), 0.2 is a factor.
distance_front_rear = 30 * 0.2


def init():
    """Initialize the simulation.
    """
    vrep.simxGetVisionSensorImage(clientID, lane_sensor, 0, vrep.simx_opmode_streaming)
    vrep.simxGetVisionSensorImage(clientID, obstacle_sensor, 0, vrep.simx_opmode_streaming)
    time.sleep(1)


def pid_controller(kp: float, ki: float, kd: float):
    """PID Control.

    :param kp: the proportional factor
    :param ki: the integral factor
    :param kd: the derivative factor
    :return: a function that processes the error
    """
    prev_error = 0
    integral = 0
    derivative = 0

    def pid(error: float):
        nonlocal prev_error
        nonlocal integral
        nonlocal derivative
        integral = integral + error
        derivative = error - prev_error
        prev_error = error
        return kp * error + ki * integral + kd * derivative

    return pid


def steer(angle: float):
    """Steer for specific angle.

    :param angle: the desired angle we want the car to steer
    """
    # angle = angle * math.pi / 180
    if angle == 0:
        angle = 1
    common = distance_front_rear / math.tan(angle)
    l_angle = math.atan(distance_front_rear / (-distance_left_right + common))
    r_angle = math.atan(distance_front_rear / (distance_left_right + common))
    _ = vrep.simxSetJointTargetPosition(clientID, l_steer, l_angle, vrep.simx_opmode_oneshot)
    _ = vrep.simxSetJointTargetPosition(clientID, r_steer, r_angle, vrep.simx_opmode_oneshot)


def motor(speed: float):
    """Set the power of the motor.

    :param speed: the desired speed
    """
    _ = vrep.simxSetJointTargetVelocity(clientID, l_motor, speed, vrep.simx_opmode_oneshot)
    _ = vrep.simxSetJointTargetVelocity(clientID, r_motor, speed, vrep.simx_opmode_oneshot)


def get_image(sensor):
    """Retrieve an image from Vision Sensor.

    :return: an image represented by numpy.ndarray from Vision Sensor
    """
    err, resolution, raw = vrep.simxGetVisionSensorImage(clientID, sensor, 0, vrep.simx_opmode_buffer)
    if err == vrep.simx_return_ok:
        image = np.array(raw, dtype=np.uint8)
        image.resize([resolution[1], resolution[0], 3])
        return image
    else:
        return None


def get_lane_image():
    """Retrieve an image from Lane Vision Sensor.

    :return: an image represented by numpy.ndarray from Lane Vision Sensor
    """
    return get_image(lane_sensor)


def get_obstacle_image():
    """Retrieve an image from Obstacle Vision Sensor.

    :return: an image represented by numpy.ndarray from Obstacle Vision Sensor
    """
    return get_image(obstacle_sensor)


def print_image(image: np.ndarray):
    """Print the image on the V-REP.

    :param image: the image we want to display on "Debug_Display"
    """
    vrep.simxSetVisionSensorImage(clientID, debug, image.ravel(), 0, vrep.simx_opmode_oneshot)


def get_contours(img: np.ndarray):
    """Get the contours of the image.

    :param img: the image we want to find its contours
    :return: the contours of the image
    """
    # Convert image to greyscale.
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Process image using Gaussian blur.
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # Process image using Color Threshold.
    _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
    # Erode and dilate to remove accidental line detections.
    mask = cv2.erode(thresh, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # Find the contours of the image.
    contours = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
    # Use imutils to unpack contours.
    contours = imutils.grab_contours(contours)
    return contours


def get_moment(contour):
    """Get the moment of the contour.

    :param contour: the contour we want to get its moment
    :return: the moment of the contour
    """
    m = cv2.moments(contour)
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    return cx, cy


def follow_lane(img: np.ndarray):
    """Return the steering angle for following lane.

    :param img: the Lane image
    :return: the steering angle
    """
    # Convert image to greyscale.
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Process image using Gaussian blur.
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # Process image using Color Threshold.
    _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
    # Erode and dilate to remove accidental line detections.
    mask = cv2.erode(thresh, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find the contours of the image.
    contours = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
    # Use imutils to unpack contours.
    contours = imutils.grab_contours(contours)
    if len(contours) == 0:
        return None

    # Find the biggest contour.
    c = max(contours, key=cv2.contourArea)
    # Get the moment of the biggest contour.
    cx, cy = get_moment(c)

    # Point out the desired moment and contour on the image.
    cv2.line(img, (cx, 0), (cx, 720), (255, 0, 0), 1)
    cv2.line(img, (0, cy), (1280, cy), (255, 0, 0), 1)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 1)

    # Calculate the angle we want to steer.
    angle = math.atan((128 - cx) / cy)

    return angle, img


def get_other_lane(img: np.ndarray):
    """Get which side the other lane is on.

    :param img: the image retrieved from the vision sensor.
    :return: which side the other lane is on, 1 means left, -1 means right
    """
    # Crop the image.
    img = img[:90, :]

    # Find the contours.
    contours = get_contours(img)
    if len(contours) < 2:
        return None

    # Sort the contours by ContourArea.
    sorted(contours, key=cv2.contourArea)
    c = contours[-1]

    # The biggest contour which doesn't have the center point is what we want.
    for i in reversed(range(len(contours))):
        if cv2.pointPolygonTest(contours[i], (128, 0), False) == -1:
            c = contours[i]

    # Get the moment of the desired contour.
    cx, cy = get_moment(c)

    # Point out the contour and the moment on the image.
    cv2.line(img, (cx, 0), (cx, 720), (255, 0, 0), 1)
    cv2.line(img, (0, cy), (1280, cy), (255, 0, 0), 1)
    cv2.drawContours(img, c, -1, (0, 255, 0), 1)

    # Determine which side the other lane is on.
    return 1 if cx < 132 else -1, img
