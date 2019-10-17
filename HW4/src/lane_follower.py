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
        raw = get_line_image()
        # Crop the line image.
        raw = raw[128:192, :]
        # Find the desired moment.
        ret = get_moment(raw)
        if ret is None:
            continue

        cx, cy, img = ret

        # Calculate the angle we want to steer.
        angle = math.atan((128 - cx) / cy)
        steer(pid(angle))

        cv2.imshow("result", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    main()
