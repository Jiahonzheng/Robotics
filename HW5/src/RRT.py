import random
from typing import List, Tuple

import cv2
import math
import numpy as np


class RRT:
    """RRT Motion Planning.
    """

    class Node:
        """RRT Node.
        """

        def __init__(self, x: float, y: float):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None  # type:None | RRT.Node

    def __init__(self, start: Tuple[int, int], goal: Tuple[int, int], obstacle_list: List[Tuple[int, int, int]],
                 rand_area: Tuple[int, int],
                 expand_dis=3.0, path_resolution=0.5, goal_sample_rate=5, max_iter=500):
        """Initialize the RRT Motion Planner.

        :param start: Start Position
        :param goal:  Goal Position
        :param obstacle_list: the list of obstacles
        :param rand_area: random sampling area
        :param expand_dis: distance of one expanding
        :param path_resolution: path resolution
        :param goal_sample_rate: gaol sample rate
        :param max_iter: max amount of iterations
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []

    def path_planning(self, img: np.ndarray):
        """Find the solution of the maze.

        :param img: the image of maze
        :return: the solution path
        """
        self.node_list = [self.start]
        for i in range(self.max_iter):
            # Generate a sample node.
            rnd_node = self.get_random_node()
            # Find the nearest node to the rnd_node.
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]
            # Expand the tree.
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
            # When there are no collisions, add new_node to the tree.
            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)
                # Show the expanding process.
                tmp = self.node_list[-1]
                x, y = int(tmp.x), int(tmp.y)
                img[x][y] = 255
                cv2.imshow("Processing", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            # Check whether it reaches the goal.
            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course()
        return None

    def steer(self, from_node: Node, to_node: Node, expand_length=float("inf")):
        """Expand the tree from from_node to to_node.
        
        :param from_node: from which node to expand
        :param to_node: to which node to expand
        :param expand_length: expand length
        :return: the new node
        """
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        if expand_length > d:
            expand_length = d
        n_expand = math.floor(expand_length / self.path_resolution)
        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
        new_node.parent = from_node
        return new_node

    def generate_final_course(self):
        """Generate the final path to Goal Position.

        :return: the final path to Goal Position
        """
        path = [(self.end.x, self.end.y)]  # type:List[Tuple[float,float]]
        node = self.node_list[len(self.node_list) - 1]
        while node.parent is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.append((node.x, node.y))
        return path[::-1]

    def calc_dist_to_goal(self, x: float, y: float):
        """Calculate the distance between node (x, y) and Goal Position.

        :param x: the x-coordinate of the node
        :param y: the y-coordinate of the node
        :return: the distance between node (x, y) and Goal Position
        """
        dx = x - self.end.x
        dy = y - self.end.y
        return math.sqrt(dx ** 2 + dy ** 2)

    def get_random_node(self):
        """Generate a random RRT.Node

        :return: a random RRT.Node
        """
        if random.randint(0, 100) > self.goal_sample_rate:  # Do random sampling.
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:  # Do goal point sampling.
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list: List[Node], rnd_node: Node):
        """Find the nearest node to rnd_node in node_list.

        :param node_list: the candidate nodes
        :param rnd_node: the target node
        :return: the nearest node to rnd_node in node_list
        """
        distances = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
        nearest = distances.index(min(distances))
        return nearest

    @staticmethod
    def check_collision(node: Node, obstacles: List[Tuple[int, int, int]]):
        """Check the node whether collides with obstacles.

        :param node: a RRT.Node
        :param obstacles: obstacles list
        :return: whether the node collides with obstacles
        """
        for (ox, oy, size) in obstacles:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]
            if min(d_list) <= size ** 2:
                return False
        return True

    @staticmethod
    def calc_distance_and_angle(from_node: Node, to_node: Node):
        """Calculate the distance and angle between two nodes.

        :param from_node: a RRT.Node
        :param to_node: a RRT.Node
        :return: the distance and angle
        """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.sqrt(dx ** 2 + dy ** 2)
        theta = math.atan2(dy, dx)
        return d, theta
