#!/usr/bin/env python
import sys
import numpy as np
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from swarm_explorer.src.utils import (
    calc_euclidean_distance,
)
from typing import Dict, Tuple

try:
    import rospy
    import tf2_ros

    from geometry_msgs.msg import Twist
    from swarm_explorer.msg import ExplorerState
except ImportError:
    pass


class TurtlebotController(object):
    def __init__(
        self,
        tb_id,
        map_obj,
        cohesion_radius,
        separation_radius,
        alignment_radius,
        collision_radius,
        cohesion_weight,
        separation_weight,
        alignment_weight,
        obstacle_weight,
        wall_weight,
        frontier_weight,
        # frontier_dist_wt,
        # frontier_size_wt,
        Kp,
        Kd,
        Ki,
    ):
        """
        Executes a plan made by the planner
        """
        self.tb_id = tb_id
        self.map_obj = map_obj
        self.state = np.array([0, 0, 0])  # (x, y, theta)
        self.state_dot = np.array([0, 0])  # (x_dot, y_dot)
        self.flock_vel = np.zeros(2)  # (linear, angular)
        self.frontier_vel = np.zeros(2)  # (linear, angular)

        self.cohesion_radius = cohesion_radius
        self.separation_radius = separation_radius
        self.alignment_radius = alignment_radius
        self.collision_radius = collision_radius
        # Weight values for the different components of the velocity
        self.cohesion_weight = cohesion_weight
        self.separation_weight = separation_weight
        self.alignment_weight = alignment_weight
        self.obstacle_weight = obstacle_weight
        self.wall_weight = wall_weight
        self.frontier_weight = frontier_weight
        # self.frontier_dist_wt = frontier_dist_wt
        # self.frontier_size_wt = frontier_size_wt

        # Gain values to be tuned
        self.Kp, self.Kd, self.Ki = Kp, Kd, Ki  # PID gains
        self.angular_gain = 1.0  # Gain for angular velocity term
        self.prev_error = np.zeros(2)  # (x, y, theta)
        self.integral = np.zeros(2)
        self.length = 0.3
        # self.neighbors = dict()

    def calc_flock_vel(self):
        """
        Calculate the flocking velocity based on the positions of the neighbors.
        """
        avg_cohesion_pos = self._calc_avg_pos_in_radius(
            self.neighbor_states, self.cohesion_radius
        )
        cohesion_vel = avg_cohesion_pos - self.state[:2]
        avg_sep_pos = self._calc_avg_pos_in_radius(
            self.neighbor_states, self.separation_radius
        )
        separation_vel = self.state[:2] - avg_sep_pos
        alignment_vel = self._calc_avg_flock_vel(
            self.neighbor_states, self.alignment_radius
        )
        x_min, x_max, y_min, y_max = self.map_obj.get_limits()
        wall_vel = np.array(
            [
                -1 if self.state[0] < x_min else 1 if self.state[0] > x_max else 0,
                -1 if self.state[1] < y_min else 1 if self.state[1] > y_max else 0,
            ]
        )

        surrounding_obstacles = self.map_obj.get_surrounding_obstacles(
            self.state[:2], self.collision_radius, is_point=True
        )
        obstacle_pos, obstacle_dist = (
            surrounding_obstacles[0] if len(surrounding_obstacles) > 0 else (None, 0)
        )

        # TODO: adjust this to be the actual x and y differences instead of just combined distance
        obstacle_vel = np.array([-obstacle_dist, -obstacle_dist])

        dynamic_scale = (self.collision_radius - obstacle_dist) / self.cohesion_radius
        self.obstacle_weight *= dynamic_scale
        self.alignment_weight *= max(1 - 2 * dynamic_scale, 0)
        self.cohesion_weight *= max(1 - 2 * dynamic_scale, 0)
        self.frontier_weight *= max(1 - 2 * dynamic_scale, 0)

        self.flock_vel = (
            self.flock_vel
            + self.cohesion_weight * cohesion_vel
            + self.alignment_weight * alignment_vel
            + self.separation_weight * separation_vel
            + self.wall_weight * wall_vel
            + self.obstacle_weight * obstacle_vel
        )  # (x_dot, y_dot)

    def calc_frontier_vel(self, target_position):
        """
        Calculate the velocity towards the target position.
        """
        best_frontier = np.argmin(self.map_obj.get_closest_frontier(self.state[:2]))

    def step_control(self, curr_odom, neighbor_states):
        """Specify a control law. For the grad/EC portion, you may want
        to edit this part to write your own closed loop controller.
        Note that this class constantly subscribes to the state of the robot,
        so the current configuratin of the robot is always stored in the
        variable self.state. You can use this as your state measurement
        when writing your closed loop controller.

        Parameters
        ----------
            target_position : target position at the current step in
                              [x, y, theta, phi] configuration space.
            open_loop_input : the prescribed open loop input at the current
                              step, as a [u1, u2] pair.
        Returns:
            None. It simply sends the computed command to the robot.
        """
        self.state = np.array(
            [
                curr_odom.pose.pose.position.x,
                curr_odom.pose.pose.position.y,
                curr_odom.twist.twist.angular.z,
            ]
        )
        self.state_dot = np.array(
            [
                curr_odom.twist.twist.linear.x,
                curr_odom.twist.twist.linear.y,
            ]
        )
        flock_vel = self.calc_flock_vel(self.neighbors)
        frontier_vel = self.calc_frontier_vel(target_position)
        target_vel = flock_vel + frontier_vel
        control_input: np.ndarray = np.array(
            [
                np.linalg.norm(target_vel),
                self.angular_gain * np.arctan2(target_vel[1], target_vel[0])
                - self.state[2],
            ]
        )

        # TODO: should I just move all command message sending into the controller class? Why not
        cmd_vel = Twist()
        cmd_vel.linear.x = control_input[0]
        cmd_vel.angular.z = control_input[1]

        return cmd_vel

    def _calc_avg_pos_in_radius(
        self, neighbor_states: Dict[int, ExplorerState], radius: float, use_actual=False
    ):
        """
        Calculate the average position of neighbors within a given radius using either actual states or calculated states.
        """
        if use_actual:
            positions = np.array(
                [
                    np.array(
                        [
                            state.odom.pose.pose.position.x,
                            state.odom.pose.pose.position.y,
                        ]
                    )
                    for state in neighbor_states.values()
                ]
            )
        else:
            positions = np.array(
                [
                    np.array(
                        [
                            state.pose.position.x,
                            state.pose.position.y,
                        ]
                    )
                    for state in neighbor_states.values()
                ]
            )
        distances = np.linalg.norm(positions - self.state[:2], axis=1)
        in_radius = positions[distances < radius]
        if len(in_radius) > 0:
            return np.mean(in_radius, axis=0)
        else:
            return np.zeros(2)

    def _calc_avg_flock_vel(
        self, neighbor_states: Dict[int, ExplorerState], radius: float
    ):
        """Calculate the average flock velocity of neighbors within a given radius."""
        velocities = np.array(
            [
                np.array(
                    [
                        state.flock_velocity.linear.x,
                        state.flock_velocity.linear.y,
                    ]
                )
                for state in neighbor_states.values()
            ]
        )
        distances = np.linalg.norm(velocities - self.state_dot[:2], axis=1)
        in_radius = velocities[distances < radius]
        if len(in_radius) > 0:
            return np.mean(in_radius, axis=0)
        else:
            return np.zeros(2)

    def plot_results(
        self,
        times,
        actual_positions,
        actual_velocities,
        target_positions,
        target_velocities,
    ):
        """
        Plots results.

        Inputs:
        times : nx' :obj:`numpy.ndarray`
        actual_positions : nx2 :obj:`numpy.ndarray`
            actual positions for each time in times
        actual_velocities: nx2 :obj:`numpy.ndarray`
            actual velocities for each time in times
        target_positions: nx2 :obj:`numpy.ndarray`
            target positions for each time in times
        target_velocities: nx2 :obj:`numpy.ndarray`
            target velocities for each time in times
        """

        # Make everything an ndarray
        times = np.array(times)
        actual_positions = np.array(actual_positions)
        actual_velocities = np.array(actual_velocities)
        target_positions = np.array(target_positions)
        target_velocities = np.array(target_velocities)

        plt.figure()
        states = ("x", "y", "theta")
        for i, state in enumerate(states):
            plt.subplot(len(states), 2, 2 * i + 1)
            plt.plot(times, actual_positions[:, state], label="Actual")
            plt.plot(times, target_positions[:, state], label="Desired")
            plt.xlabel("Time (t)")
            plt.ylabel(states[state] + " Position Error")
            plt.legend()

            plt.subplot(i, 2, 2 * i + 2)
            plt.plot(times, actual_velocities[:, state], label="Actual")
            plt.plot(times, target_velocities[:, state], label="Desired")
            plt.xlabel("Time (t)")
            plt.ylabel(states[i] + " Velocity Error")
            plt.legend()

        plt.show()
