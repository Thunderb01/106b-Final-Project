#!/usr/bin/env python3
import rospy
import numpy as np
from typing import Tuple, Dict, Any

class EnvConfig:
    """
    Environment configuration class that loads parameters from ROS parameter server.
    """
    def __init__(self):
        """Initialize environment configuration from ROS parameters."""
        # Load parameters from ROS parameter server
        self._load_params()

    def _load_params(self):
        """Load all parameters from ROS parameter server."""
        # Map settings
        self.map_resolution = rospy.get_param("~env/map_resolution", 0.05)
        self.map_width = rospy.get_param("~env/map_width", 100)
        self.map_height = rospy.get_param("~env/map_height", 100)
        self.map_origin_x = rospy.get_param("~env/map_origin_x", -2.5)
        self.map_origin_y = rospy.get_param("~env/map_origin_y", -2.5)

        # Robot motion limits
        self.max_linear_vel = rospy.get_param("~env/max_linear_vel", 0.22)
        self.max_angular_vel = rospy.get_param("~env/max_angular_vel", 2.84)
        self.max_linear_acc = rospy.get_param("~env/max_linear_acc", 0.5)
        self.max_angular_acc = rospy.get_param("~env/max_angular_acc", 3.2)

        # Robot dimensions
        self.robot_radius = rospy.get_param("~env/robot_radius", 0.1)
        self.robot_length = rospy.get_param("~env/robot_length", 0.3)

        # Flocking parameters
        self.cohesion_radius = rospy.get_param("~env/cohesion_radius", 1.0)
        self.separation_radius = rospy.get_param("~env/separation_radius", 0.5)
        self.alignment_radius = rospy.get_param("~env/alignment_radius", 1.0)
        self.collision_radius = rospy.get_param("~env/collision_radius", 0.5)

        # Weights for different behaviors
        self.cohesion_weight = rospy.get_param("~env/cohesion_weight", 0.23)
        self.separation_weight = rospy.get_param("~env/separation_weight", 1.1)
        self.alignment_weight = rospy.get_param("~env/alignment_weight", 0.5)
        self.obstacle_weight = rospy.get_param("~env/obstacle_weight", 1.1)
        self.wall_weight = rospy.get_param("~env/wall_weight", 1.1)
        self.frontier_weight = rospy.get_param("~env/frontier_weight", 0.08)

        # Frontier parameters
        self.frontier_dist_wt = rospy.get_param("~env/frontier_dist_wt", 0.001)
        self.frontier_size_wt = rospy.get_param("~env/frontier_size_wt", 1.0)

        # Communication parameters
        self.comm_radius = rospy.get_param("~env/comm_radius", 5.0)
        self.max_neighbor_age = rospy.get_param("~env/max_neighbor_age", 0.5)

    def normalize_position(self, pos: np.ndarray) -> np.ndarray:
        """
        Normalize a position to the map frame.
        Args:
            pos: Position in world coordinates [x, y]
        Returns:
            Normalized position in map coordinates [x, y]
        """
        return (pos - np.array([self.map_origin_x, self.map_origin_y])) / self.map_resolution

    def denormalize_position(self, pos_norm: np.ndarray) -> np.ndarray:
        """
        Convert a normalized position back to world coordinates.
        Args:
            pos_norm: Position in map coordinates [x, y]
        Returns:
            Position in world coordinates [x, y]
        """
        return pos_norm * self.map_resolution + np.array([self.map_origin_x, self.map_origin_y])

    def clip_velocity(self, vel: np.ndarray) -> np.ndarray:
        """
        Clip velocity to ensure it stays within limits.
        Args:
            vel: Velocity [linear, angular]
        Returns:
            Clipped velocity [linear, angular]
        """
        linear_vel = np.clip(vel[0], -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(vel[1], -self.max_angular_vel, self.max_angular_vel)
        return np.array([linear_vel, angular_vel])

    def get_map_limits(self) -> Tuple[float, float, float, float]:
        """
        Get the map limits in world coordinates.
        Returns:
            Tuple of (x_min, x_max, y_min, y_max)
        """
        x_min = self.map_origin_x
        x_max = x_min + self.map_width * self.map_resolution
        y_min = self.map_origin_y
        y_max = y_min + self.map_height * self.map_resolution
        return x_min, x_max, y_min, y_max

    def get_map_info(self) -> Dict[str, Any]:
        """
        Get map information for visualization.
        Returns:
            Dictionary containing map information
        """
        return {
            "resolution": self.map_resolution,
            "width": self.map_width,
            "height": self.map_height,
            "origin_x": self.map_origin_x,
            "origin_y": self.map_origin_y
        } 