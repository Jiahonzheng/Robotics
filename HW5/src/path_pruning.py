from typing import List, Tuple

import cv2
import numpy as np

from utils import get_obstacles, draw_path


def path_pruning(path: np.ndarray, obstacles: List[Tuple[int, int, int]]):
    """Prune the solution path.

    :param path: the solution path
    :return: the pruned path
    """
    pruned_path = [path[0]]
    print(pruned_path)
    return path


def test():
    """Test: Path Pruning.
    """
    # Load the maze image.
    img = cv2.imread("maze.png", cv2.THRESH_BINARY)  # type:np.ndarray

    # Load and draw the solution path.
    path = np.loadtxt("solution.txt")
    cv2.imshow("Original Solution", draw_path(img.copy(), path))

    # Prune the solution path.
    pruned_path = path_pruning(path, get_obstacles(img))
    cv2.imshow("Pruned Solution", draw_path(img.copy(), pruned_path))

    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    test()
