from path_following import path_following
from path_planning import path_planning
from remote import *
from utils import *


def main():
    """Find the solution and follow it.
    """
    # Initialize the simulation.
    init()

    # Retrieve the maze image.
    maze = get_image(vision_sensor)  # type: np.ndarray
    cv2.imshow("Maze", maze)
    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Find the solution.
    start = time.time()
    solution = path_planning(maze)
    cost = time.time() - start
    print(f"It costs {cost} seconds to find the path.")

    # Show the solution.
    cv2.imshow("Solution", draw_path(maze, solution))
    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Convert the coordinates.
    path = []
    for i in range(len(solution)):
        path.append(world_coordinate(solution[i]))

    # Follow the path.
    path_following(path)


if __name__ == '__main__':
    main()
