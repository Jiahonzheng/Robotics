import time

from RRT import *
from utils import get_obstacles, draw_path


def path_planning(img: np.ndarray):
    """Find the solution of the maze.

    :param img: the image of the maze
    :return: the solution
    """
    # Define Start Position and Goal Position.
    start = (30, 90)
    goal = (480, 480)
    obstacles = get_obstacles(img)
    # Initialize RRT Motion Planner.
    rrt = RRT(start, goal, obstacles, (50, 480), expand_dis=20, goal_sample_rate=35, path_resolution=10, max_iter=50000)
    # Plan the path.
    path = rrt.path_planning(img.copy())
    if path is None:
        raise Exception("Cannot find the path.")
    # Save the solution.
    np.savetxt("solution.txt", np.array(path), fmt="%s")
    return path


def main():
    """Path Planning.
    """
    # Load the maze image.
    img = cv2.imread("maze.png", cv2.THRESH_BINARY)  # type:np.ndarray
    cv2.imshow("Maze", img)

    # Find the solution.
    start = time.time()
    solution = path_planning(img)
    cost = time.time() - start
    print(f"It costs {cost} seconds to find the path.")

    # Show the solution.
    cv2.imshow("Solution", draw_path(img, solution))
    cv2.imwrite("solution.png", draw_path(img, solution))

    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    main()
