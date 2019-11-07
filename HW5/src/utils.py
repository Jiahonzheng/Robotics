from typing import Tuple

import math
import numpy as np


def get_obstacles(img: np.ndarray):
    """Return the list of obstacles.

    :param img: the maze image
    :return: the list of obstacles
    """
    obstacles = []
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            if img[i][j] == 255:
                obstacles.append((i, j, 15))
    return obstacles


def draw_path(img: np.ndarray, path: np.ndarray):
    """Draw the path on the maze image.

    :param img: the maze image
    :param path: the solution path
    :return: the maze image with the solution path
    """
    for i in range(len(path)):
        x, y = int(path[i][0]), int(path[i][1])
        img[x][y] = 255
    return img


def distance(x1: float, y1: float, x2: float, y2: float):
    """Calculate the distance between (x1, y1) and (x2, y2).

    :param x1: x-coordinate of (x1, y1)
    :param y1: y-coordinate of (x1, y1)
    :param x2: x-coordinate of (x2, y2)
    :param y2: y-coordinate of (x2, y2)
    :return: the distance between (x1, y1) and (x2, y2)
    """
    dx = x1 - x2
    dy = y1 - y2
    return math.sqrt(dx * dx + dy * dy)


def world_coordinate(coord: Tuple[float, float]):
    """Convert Sensor-Coordinate to World-Coordinate.

    :param coord: Coordinate related to Vision Sensor
    :return: World-Coordinate
    """
    k = 64 * math.sqrt(3)
    x, y = (256 - coord[0]) / k, (coord[1] - 256) / k + 0.8
    return x, y
