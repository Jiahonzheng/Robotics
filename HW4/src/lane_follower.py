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
        ret = follow_lane(lane_raw)
        if ret is None:
            continue

        # Steer for calculated angle.
        angle, img = ret
        steer(pid(angle))

        # Show the processed image.
        cv2.imshow("result", img)

        # When we press "q", it quits the simulation.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    main()
