from typing import List

from remote import *
from utils import *


def path_following(path: List[Tuple[int, int]]):
    """Follow the solution path.

    :param path: the solution path
    """
    i = 1
    while vrep.simxGetConnectionId(clientID) != -1:
        _, cur = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
        # Calculate the distance and the angle.
        dis = distance(cur[0], cur[1], path[i][0], path[i][1])
        phi = math.atan2(path[i][1] - cur[1], path[i][0] - cur[0])


def main():
    """Path Following.
    """
    # Initialize the simulation.
    init()

    # Load the solution path.
    solution = np.loadtxt("pruned_solution.txt")
    # Convert the coordinates.
    path = []
    for i in range(len(solution)):
        path.append(world_coordinate(solution[i]))

    # Follow the solution path.
    path_following(path)


if __name__ == '__main__':
    main()
