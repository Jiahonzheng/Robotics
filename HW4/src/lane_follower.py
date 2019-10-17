from utils import *


def main():
    # Initialize the simulation.
    init()

    # PID Parameters
    # Motor Kp Ki Kd
    # 5 0.01 0 0
    # 10 0.0001 0.0005

    pid = pid_controller(1, 0.0001, 0.0005)
    # Set the Power!
    motor(10)

    # Start the main loop.
    while vrep.simxGetConnectionId(clientID) != -1:
        # Retrieve the image from Lane Vision Sensor.
        lane_raw = get_lane_image()
        if lane_raw is None:
            continue

        # Crop the lane image.
        lane_raw = lane_raw[128:192, :]
        # Find the desired moment.
        ret = get_moment(lane_raw)
        if ret is None:
            continue

        cx, cy, lane_img = ret

        # Calculate the angle we want to steer.
        angle = math.atan((128 - cx) / cy)

        # Steer for calculated angle.
        steer(pid(angle))

        cv2.imshow("result", lane_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    main()
