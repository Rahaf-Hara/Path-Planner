import rospy
import numpy as np
import copy
from geometry_msgs.msg import Point
from .utils import is_occluded

class PathSmoother():
    def __init__(self, graph, path):
        self.graph_ = graph
        self.path_ = self.smooth_path(path)
        self.graph_.visualize_path_smooth(self.path_)

    def smooth_path(self, path_nodes):
        """Smooths a given path using an iterative adjustment based on local path relaxation."""
        path = [Point(node.x, node.y, 0) for node in path_nodes]
        path_smooth = copy.deepcopy(path)

        alpha = rospy.get_param("~alpha")
        beta = rospy.get_param("~beta")
        epsilon = 0.001
        change = epsilon  # Initialize change to enter the loop

        while change >= epsilon:
            change = 0
            for i in range(1, len(path)-1):
                old_x, old_y = path_smooth[i].x, path_smooth[i].y

                # Update coordinates based on relaxation formula
                path_smooth[i].x = (1 - alpha - 2 * beta) * path_smooth[i].x + alpha * path[i].x + beta * (path_smooth[i - 1].x + path_smooth[i + 1].x)
                path_smooth[i].y = (1 - alpha - 2 * beta) * path_smooth[i].y + alpha * path[i].y + beta * (path_smooth[i - 1].y + path_smooth[i + 1].y)

                # Check if the updated path goes through an obstacle

                if is_occluded(self.graph_.map_.image_, [path_smooth[i-1].x, path_smooth[i-1].y], [path_smooth[i].x, path_smooth[i].y]) or is_occluded(self.graph_.map_.image_, [path_smooth[i].x, path_smooth[i].y], [path_smooth[i+1].x, path_smooth[i+1].y]):
                    # If the path goes through an obstacle, revert the changes
                    path_smooth[i].x, path_smooth[i].y = old_x, old_y

                change += np.linalg.norm([path_smooth[i].x - old_x, path_smooth[i].y - old_y])

            if change < epsilon:
                break

        return path_smooth
