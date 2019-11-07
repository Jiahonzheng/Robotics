from typing import List

import cv2

from utils import *


def path_pruning(path: np.ndarray, obstacles: List[Tuple[int, int, int]]):
    """Prune the solution path.

    :param path: the solution path
    :param obstacles: the list of obstacles
    :return: the pruned path
    """
    pruned_path = [path[0]]
    n, m = len(path), len(obstacles)
    cur = 0
    th = obstacles[0][2]
    # 将连线进行 n 等分操作。
    sz = 7
    while True:
        if cur == n - 1:
            break
        to = cur + 1
        for j in range(cur + 1, min(cur + 6, n)):
            x1, y1 = path[cur][0], path[cur][1]
            x2, y2 = path[j][0], path[j][1]
            ok = True
            for h in range(sz):
                mx, my = (h * x1 + (sz - h) * x2) / sz, (h * y1 + (sz - h) * y2) / sz
                min_dist = 99999999
                for k in range(m):
                    x, y = obstacles[k][0], obstacles[k][1]
                    min_dist = min(min_dist, distance(x, y, mx, my))
                if min_dist <= th:
                    ok = False
                    break
            if ok:
                to = max(to, j)
        pruned_path.append(path[to])
        cur = to
    ret = np.array(pruned_path)
    np.savetxt("pruned_solution.txt", ret)
    return ret


def main():
    """Path Pruning.
    """
    # Load the maze image.
    img = cv2.imread("maze.png", cv2.THRESH_BINARY)  # type:np.ndarray

    # Load and draw the solution path.
    path = np.loadtxt("solution.txt")
    cv2.imshow("Original Solution", draw_path(img.copy(), path))

    # Prune the solution path.
    pruned_path = path_pruning(path, get_obstacles(img))
    cv2.imshow("Pruned Solution", draw_path(img.copy(), pruned_path))
    cv2.imwrite("pruned_solution.png", draw_path(img.copy(), pruned_path))

    for i in range(len(pruned_path)):
        print(world_coordinate(pruned_path[i]))

    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    main()
