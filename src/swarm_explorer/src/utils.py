"""
Utility functions for swarm explorer.
"""

import numpy as np
from typing import Dict, Tuple

try:
    import rospy
    from nav_msgs.msg import Odometry
except ImportError:
    pass


def calc_euclidean_distance(p1: Odometry, p2: Odometry):
    """
    Calculate the Euclidean distance between two points.
    """
    p1 = np.array([p1.pose.pose.position.x, p1.pose.pose.position.y])
    p2 = np.array([p2.pose.pose.position.x, p2.pose.pose.position.y])
    return np.linalg.norm(p1 - p2)


def calc_angle_between_points(p1: Odometry, p2: Odometry) -> float:
    """
    Calculate the angle between two points.
    """
    p1 = np.array([p1.pose.pose.position.x, p1.pose.pose.position.y])
    p2 = np.array([p2.pose.pose.position.x, p2.pose.pose.position.y])
    return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
