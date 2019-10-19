from utils import *


def main():
    # Initialize the simulation.
    init()

    # PID Parameters
    # Motor Kp Ki Kd
    # 10 1 0.00001 0.0001

    pid = pid_controller(1, 0.00001, 0.0001)
    # Set the Power!
    motor(10)

    # It is used to avoid obstacle.
    avoid_obstacle_time = 0

    # Start the main loop.
    while vrep.simxGetConnectionId(clientID) != -1:
        simulation_time = vrep.simxGetLastCmdTime(clientID)
        if simulation_time - avoid_obstacle_time <= 1000:
            continue

        # Read the Proximity Sensor to determine whether we detect the obstacle.
        ret = vrep.simxReadProximitySensor(clientID, proximity_sensor, vrep.simx_opmode_streaming)
        if ret[1] is False:  # When there is no obstacle.
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
            cv2.imshow("Lane", img)
        else:  # When there is an obstacle.
            obstacle_raw = get_obstacle_image()
            if obstacle_raw is None:
                continue

            # Get the other lane.
            ret = get_other_lane(obstacle_raw.copy())
            if ret is None:
                continue

            # Steer to enter other lane to avoid obstacle.
            direction, img = ret
            if direction == 1:  # Turn left.
                steer(1 / 6 * math.pi)
            else:  # Turn right.
                steer(-1 / 6 * math.pi)

            # Set the time for avoiding obstacle.
            avoid_obstacle_time = simulation_time

            # Show the processed image.
            cv2.imshow("Other Lane", img)

        # When we press "q", it quits the simulation.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    main()
