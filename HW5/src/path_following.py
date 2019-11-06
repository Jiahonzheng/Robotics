from typing import List

from remote import *
from utils import *


def path_following(path: List[Tuple[int, int]]):
    """Follow the solution path.

    :param path: the solution path
    """
    i = 1
    stage = 0
    goal_angle = None
    tolerance = 2
    while vrep.simxGetConnectionId(clientID) != -1:
        _, cur = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)

        if i == len(path):
            break

        if stage == 0:
            cur_angle = get_beta_angle()
            if goal_angle is None:
                phi = math.atan2(path[i][0] - cur[0], path[i][1] - cur[1])
                goal_angle = -math.degrees(phi)
            delta = cur_angle - goal_angle
            if delta > tolerance:
                move(0, -0.1)
                continue
            if delta < -tolerance:
                move(0, 0.1)
                continue
            goal_angle = None
            move(0, 0)
            stage = 1
            continue

        if stage == 1:
            dis = distance(cur[0], cur[1], path[i][0], path[i][1])
            if dis < 0.1:
                print(path[i])
                i += 1
                stage = 0
                move(0, 0)
                continue
            move(0.1, 0)
            continue


def main():
    """Path Following.
    """
    # Initialize the simulation.
    init()

    # Load the solution path.
    solution = np.loadtxt("solution.txt")
    # Convert the coordinates.
    path = []
    for i in range(len(solution)):
        path.append(world_coordinate(solution[i]))

    # Follow the solution path.
    path_following(path)


if __name__ == '__main__':
    main()
