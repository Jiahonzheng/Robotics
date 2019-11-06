from utils import *


def main():
    # Initialize the simulation.
    init()

    # Start the main loop.
    while vrep.simxGetConnectionId(clientID) != -1:
        # Retrieve the image from Vision Sensor.
        img = get_image(vision_sensor)
        cv2.imshow("Result", img)
        # When we press "q", it quits the simulation.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.imwrite("maze.png", img)
            break


if __name__ == '__main__':
    main()
