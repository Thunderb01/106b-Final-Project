#!/usr/bin/env python
from typing import Dict, Tuple
import numpy as np
import matplotlib
import sys

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from swarm_explorer.src.utils import (
    calc_euclidean_distance,
    wrap_angle,
)

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
        # map_obj,
        neighbor_states,
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
        Kp,
        Kd,
        Ki,
    ):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher(f"/robot_{tb_id}/cmd_vel", Twist, queue_size=1)
        self.tb_id = tb_id
        self.state = np.array([0, 0, 0])  # (x, y, theta)
        self.state_dot = np.array([0, 0, 0])  # (x_dot, y_dot, theta_dot)
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
        # init velocity components
        self.cohesion_vel = np.zeros(2)  # (x_dot, y_dot)
        self.separation_vel = np.zeros(2)  # (x_dot, y_dot)
        self.alignment_vel = np.zeros(2)  # (x_dot, y_dot)
        self.wall_vel = np.zeros(2)  # (x_dot, y_dot)
        self.obstacle_vel = np.zeros(2)  # (x_dot, y_dot)
        self.frontier_vel = np.zeros(2)  # (x_dot, y_dot)

        # Stored values for PID controller
        self.prev_time = 0.0
        self.integ_sum = np.zeros(2)  # (x, theta)
        self.prev_error = np.zeros(2)  # (x, theta)
        # Gain values to be tuned
        self.Kp, self.Kd, self.Ki = Kp, Kd, Ki  # PID gains
        self.angular_gain = 1.0  # Gain for angular velocity term
        self.prev_error = np.zeros(2)  # (x, y, theta)
        self.integral = np.zeros(2)
        self.length = 0.3

        # Plotting variables
        self.times = []  # Time stamps for plotting
        self.actual_states = []  # (x, y, theta)
        self.target_states = []  # (x, y, theta)
        self.actual_velocities = []  # (linear, angular)
        self.target_velocities = []  # (linear, angular)

    def calc_flock_vel(self, latest_map, neighbor_states):
        """
        Calculate the flocking velocity based on the positions of the neighbors.
        """
        avg_cohesion_pos = self._calc_avg_pos_in_radius(
            neighbor_states, self.cohesion_radius
        )
        self.cohesion_vel = avg_cohesion_pos - self.state[:2]
        avg_sep_pos = self._calc_avg_pos_in_radius(
            neighbor_states, self.separation_radius
        )
        self.separation_vel = self.state[:2] - avg_sep_pos
        self.alignment_vel = self._calc_avg_flock_vel(
            neighbor_states, self.alignment_radius
        )
        x_min, x_max, y_min, y_max = latest_map.get_limits()
        self.wall_vel = np.array(
            [
                -1 if self.state[0] < x_min else 1 if self.state[0] > x_max else 0,
                -1 if self.state[1] < y_min else 1 if self.state[1] > y_max else 0,
            ]
        )

        surrounding_obstacles = latest_map.get_surrounding_obstacles(
            self.state[:2], self.collision_radius, is_point=True
        )
        obstacle_pos, obstacle_dist = (
            surrounding_obstacles[0] if len(surrounding_obstacles) > 0 else (None, 0)
        )

        if obstacle_pos is not None:
            self.obstacle_vel = np.array(
                [self.state[0] - obstacle_pos[0], self.state[1] - obstacle_pos[1]]
            )
        else:
            self.obstacle_vel = np.zeros(2)

        dynamic_scale = (self.collision_radius - obstacle_dist) / self.cohesion_radius
        self.obstacle_weight *= dynamic_scale
        self.alignment_weight *= max(1 - 2 * dynamic_scale, 0)
        self.cohesion_weight *= max(1 - 2 * dynamic_scale, 0)
        self.frontier_weight *= max(1 - 2 * dynamic_scale, 0)

        self.flock_vel = (
            self.flock_vel
            + self.cohesion_weight * self.cohesion_vel
            + self.alignment_weight * self.alignment_vel
            + self.separation_weight * self.separation_vel
            + self.wall_weight * self.wall_vel
            + self.obstacle_weight * self.obstacle_vel
        )  # (x_dot, y_dot)

    def calc_frontier_vel(self, best_frontier):
        """
        Calculate the velocity towards the target position.
        """
        self.frontier_vel = (
            self.frontier_weight
            * (np.array(best_frontier.get_centroid()) - self.state[:2])
            * (np.isclose(np.linalg.norm(self.separation_vel), 0.0))
        )

    def calc_reference_vels(
        self, curr_state, latest_map, neighbor_states, best_frontier
    ) -> np.ndarray:
        """
        Calculate the control input based on Boyd's model. Update the robot's
        state and velocities.
        Parameters
        ----------
            curr_state : current state of the robot in [x, y, theta] configuration space.
            latest_map : latest map of the environment.
            neighbor_states : states of the neighboring robots.
            best_frontier : best frontier to explore.
        Returns
            ref_velocities : reference velocities for the robot in [linear, angular] configuration space.
        """
        # Update the robot's velocities
        self.state = curr_state
        self.calc_flock_vel(latest_map=latest_map, neighbor_states=neighbor_states)
        self.calc_frontier_vel(best_frontier=best_frontier)
        target_vel = self.flock_vel + self.frontier_vel  # (x_dot, y_dot)
        target_theta = np.arctan2(target_vel[1], target_vel[0])
        theta_dot = wrap_angle(target_theta - self.state[2])
        self.state[:2] += target_vel
        self.state[2] = target_theta
        self.state_dot[:2] = target_vel
        self.state[2] = theta_dot
        ref_velocities: np.ndarray = np.array(
            [np.linalg.norm(target_vel), self.angular_gain * theta_dot]
        )  # (linear, angular)
        return ref_velocities

    def step_control(self, target_state, curr_odom):
        """Specify a control law. For the grad/EC portion, you may want
        to edit this part to write your own closed loop controller.
        Note that this class constantly subscribes to the state of the robot,
        so the current configuratin of the robot is always stored in the
        variable self.state. You can use this as your state measurement
        when writing your closed loop controller.

        Parameters
        ----------
            target_state : target state at the current step in
                              [linear, angular] configuration space.
            curr_odom : current odometry of the robot.
        Returns:
            None. It simply sends the computed command to the robot.
        """
        control_input = Twist()
        # PID Controller
        actual_state = np.array(
            [
                curr_odom.twist.twist.linear.x,
                curr_odom.twist.twist.angular.z,
            ]
        )
        # x_error = target_state[0] - actual_state[0]
        # theta_error = target_state[1] - actual_state[1]
        error = target_state - actual_state

        # Proportional term
        proportional = self.Kp @ error
        # Integral term
        curr_time = curr_odom.header.stamp.to_sec()
        dt = curr_time - self.prev_time
        self.integ_sum += error * dt
        integral = self.Ki @ self.integ_sum
        # Derivative term
        error_deriv = (error - self.prev_error) / dt
        self.prev_error = error
        derivative = self.Kd @ error_deriv
        self.prev_time = curr_time

        control_input.linear.x = proportional[0] + derivative[0] + integral[0]
        control_input.angular.z = proportional[1] + derivative[1] + integral[1]
        self.cmd(control_input)

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

    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim

        Parameters
        ----------
        msg : Twist
        """
        self.pub.publish(msg)

    def shutdown(self):
        """
        Shutdown the node.
        """
        rospy.loginfo("Shutting down")
        self.cmd(Twist())

    def plot_results(self):
        """
        Plots results.

        times : nx' :obj:`numpy.ndarray`
        actual_states : nx2 :obj:`numpy.ndarray`
            actual positions for each time in times
        actual_velocities: nx2 :obj:`numpy.ndarray`
            actual velocities for each time in times
        target_states: nx2 :obj:`numpy.ndarray`
            target positions for each time in times
        target_velocities: nx2 :obj:`numpy.ndarray`
            target velocities for each time in times
        """

        # Make everything an ndarray
        times = np.array(self.times)
        actual_states = np.array(self.actual_states)
        actual_velocities = np.array(self.actual_velocities)
        target_states = np.array(self.target_states)
        target_velocities = np.array(self.target_velocities)

        plt.figure()
        states = ("x", "y", "theta")
        for i, state in enumerate(states):
            plt.subplot(len(states), 2, 2 * i + 1)
            plt.plot(times, actual_states[:, i], label="Actual")
            plt.plot(times, target_states[:, i], label="Desired")
            plt.xlabel("Time (t)")
            plt.ylabel(states[state] + " State Error")
            plt.legend()

            plt.subplot(i, 2, 2 * i + 2)
            plt.plot(times, actual_velocities[:, i], label="Actual")
            plt.plot(times, target_velocities[:, i], label="Desired")
            plt.xlabel("Time (t)")
            plt.ylabel(states[i] + " Velocity Error")
            plt.legend()

        plt.show()
