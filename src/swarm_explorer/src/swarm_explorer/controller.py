#!/usr/bin/env python
from typing import Dict, Tuple
import numpy as np
import matplotlib
import sys
import tf_conversions
import os

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from swarm_explorer.utils import (
    calc_euclidean_distance,
    wrap_angle,
)

try:
    import rospy
    import tf2_ros

    from geometry_msgs.msg import Twist
    from swarm_explorer.msg import ExplorerStateMsg
except ImportError:
    pass


class TurtlebotController(object):
    def __init__(
        self,
        tb_id,
        # map_obj,
        # neighbor_states,
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
        env_config,
    ):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher(f"/robot_{tb_id}/cmd_vel", Twist, queue_size=1)
        self.tb_id = tb_id
        self.state = np.array([0, 0, 0])  # (x, y, theta)
        self.state_dot = np.array([0, 0, 0])  # (x_dot, y_dot, theta_dot)
        self.flock_vel = np.zeros(2)  # (linear_x, linear_y)
        self.frontier_vel = np.zeros(2)  # (linear_x, linear_y)

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
        self.env_config = env_config
        
        # Initialize plotting variables
        self.actual_positions = []
        self.actual_velocities = []
        self.target_positions = []
        self.target_velocities = []
        self.times = []
        self.start_time = rospy.Time.now()

    def _calc_obstacle_vel(self, latest_map):
        """
        Calculate the velocity towards the nearest obstacle and velcoity to avoid walls.
        """
        surrounding_obstacles = latest_map.get_surrounding_obstacles(
            self.state[:2], self.collision_radius, is_point=True
        )
        rospy.logwarn("Obstacles: %s", surrounding_obstacles)
        obstacle_pos, obstacle_dist = (
            surrounding_obstacles[0] if len(surrounding_obstacles) > 0 else (None, 0)
        )
        if obstacle_pos is not None:
            self.obstacle_vel = np.array(
                [self.state[0] - obstacle_pos[0], self.state[1] - obstacle_pos[1]]
            )
            self.obstacle_vel = self.obstacle_vel / np.linalg.norm(self.obstacle_vel)
        else:
            self.obstacle_vel = np.zeros(2)
            self.obstacle_weight = 0.0
        self.flock_vel += self.obstacle_weight * self.obstacle_vel

    def _calc_flock_vel(self, latest_map, neighbor_states):
        """
        Calculate the flocking velocity based on the positions of the neighbors.
        """
        # Wall velocity
        x_min, x_max, y_min, y_max = latest_map.get_limits()
        self.wall_vel = np.array(
            [
                -1 if self.state[0] < x_min else 1 if self.state[0] > x_max else 0,
                -1 if self.state[1] < y_min else 1 if self.state[1] > y_max else 0,
            ]
        )
        self.flock_vel += self.wall_weight * self.wall_vel
        
        # Obstacle velocity
        rospy.logwarn("Calculating obstacle velocity")
        surrounding_obstacles = latest_map.get_surrounding_obstacles(
            self.state[:2], self.collision_radius, is_point=True
        )
        rospy.logwarn("Obstacles: %s", surrounding_obstacles)
        obstacle_pos, obstacle_dist = (
            surrounding_obstacles[0] if len(surrounding_obstacles) > 0 else (None, 0)
        )
        if obstacle_pos is not None:
            self.obstacle_vel = np.array(
                [self.state[0] - obstacle_pos[0], self.state[1] - obstacle_pos[1]]
            )
            self.obstacle_vel = self.obstacle_vel / np.linalg.norm(self.obstacle_vel)
        else:
            self.obstacle_vel = np.zeros(2)
            self.obstacle_weight = 0.0
        self.flock_vel += self.obstacle_weight * self.obstacle_vel

        # Flock Velocity Calculation if neighbors
        if len(neighbor_states) == 0:
            rospy.logwarn("Robot %d: no neighbors found", self.tb_id)
            self.cohesion_vel = np.zeros(2)
            self.separation_vel = np.zeros(2)
            self.alignment_vel = np.zeros(2)
        else:    
            # Cohesion velocity
            avg_cohesion_pos = self._calc_avg_pos_in_radius(
                neighbor_states, self.cohesion_radius
            )
            self.cohesion_vel = avg_cohesion_pos - self.state[:2]
            self.cohesion_vel = self.cohesion_vel / np.linalg.norm(self.cohesion_vel)

            # Separation velocity
            avg_sep_pos = self._calc_avg_pos_in_radius(
                neighbor_states, self.separation_radius
            )
            self.separation_vel = self.state[:2] - avg_sep_pos
            self.separation_vel = self.separation_vel / np.linalg.norm(self.separation_vel)

            # Alignment velocity
            self.alignment_vel = self._calc_avg_flock_vel(
                neighbor_states, self.alignment_radius
            )
            self.alignment_vel = self.alignment_vel / np.linalg.norm(self.alignment_vel)

        dynamic_scale = (self.collision_radius - obstacle_dist) / self.cohesion_radius
        self.obstacle_weight *= dynamic_scale
        self.alignment_weight *= max(1 - 2 * dynamic_scale, 0)
        self.cohesion_weight *= max(1 - 2 * dynamic_scale, 0)
        self.frontier_weight *= max(1 - 2 * dynamic_scale, 0)

        self.flock_vel = flock_vel = (
            self.flock_vel
            # + self.cohesion_weight * self.cohesion_vel
            + self.alignment_weight * self.alignment_vel
            + self.separation_weight * self.separation_vel
            + self.wall_weight * self.wall_vel
            + self.obstacle_weight * self.obstacle_vel
        )  # (x_dot, y_dot)
        # rospy.loginfo(f"Flock velocity: {flock_vel}")

    def _calc_frontier_vel(self, best_frontier):
        """
        Calculate the velocity towards the target position.
        """
        if best_frontier is None:
            self.frontier_vel = np.zeros(2)
            return
        self.frontier_vel = (
            self.frontier_weight
            * (np.array(best_frontier.get_centroid()) - self.state[:2])
            * (np.isclose(np.linalg.norm(self.separation_vel), 0.0))
        )
        # rospy.loginfo(f"Frontier velocity: {self.frontier_vel}")

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
        # rospy.loginfo(f"Calculating reference velocities for robot {self.tb_id}")
        self.state = curr_state
        self._calc_flock_vel(latest_map=latest_map, neighbor_states=neighbor_states)
        # rospy.loginfo(f"Flock velocity: {self.flock_vel}")
        self._calc_frontier_vel(best_frontier=best_frontier)
        # rospy.loginfo(f"Frontier velocity: {self.frontier_vel}")
        target_vel = self.flock_vel + self.frontier_vel  # (x_dot, y_dot)
        # rospy.loginfo(f"Target velocity: {target_vel}")
        
        # Normalize target velocity to unit vector
        target_vel_mag = np.linalg.norm(target_vel)
        if target_vel_mag > 0:
            target_vel = target_vel / target_vel_mag
            
        # Scale to maximum allowed velocity
        target_vel = target_vel * self.env_config.max_linear_vel
        
        target_theta = np.arctan2(target_vel[1], target_vel[0])
        theta_dot = wrap_angle(target_theta - self.state[2])
        self.state[:2] += target_vel
        self.state[2] = target_theta
        self.state_dot[:2] = target_vel
        self.state[2] = theta_dot
        
        # Calculate reference velocities with normalized linear velocity
        ref_velocities: np.ndarray = np.array(
            [np.linalg.norm(target_vel), self.angular_gain * theta_dot]
        )  # (linear, angular)
        
        # Clip velocities to ensure they stay within limits
        ref_velocities = self.env_config.clip_velocity(ref_velocities)
        
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
        error = target_state - actual_state

        # Proportional term
        proportional = self.Kp @ error
        # Integral term
        curr_time = curr_odom.header.stamp.to_sec()
        dt = curr_time - self.prev_time
        self.integ_sum += error * dt
        integral = self.Ki @ self.integ_sum
        # Derivative term
        if np.isclose(dt, 0.0):
            derivative = np.zeros(2)
        else:
            error_deriv = (error - self.prev_error) / dt
            derivative = self.Kd @ error_deriv
        self.prev_error = error
        self.prev_time = curr_time

        control_input.linear.x = proportional[0] + derivative[0] + integral[0]
        control_input.angular.z = proportional[1] + derivative[1] + integral[1]
        self.cmd(control_input)

        # Update plotting variables
        current_time = (rospy.Time.now() - self.start_time).to_sec()
        self.times.append(current_time)
        
        # Get current position and velocity
        curr_pos = np.array([curr_odom.pose.pose.position.x, curr_odom.pose.pose.position.y])
        curr_vel = np.array([curr_odom.twist.twist.linear.x, curr_odom.twist.twist.angular.z])
        
        # Get target velocity (target_state is already in [linear, angular] format)
        target_vel = target_state
        
        # Store for plotting
        self.actual_positions.append(curr_pos)
        self.actual_velocities.append(curr_vel)
        self.target_velocities.append(target_vel)

    def _calc_avg_pos_in_radius(
        self, neighbor_states: Dict[int, ExplorerStateMsg], radius: float, use_actual=False
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
        self, neighbor_states: Dict[int, ExplorerStateMsg], radius: float
    ):
        """Calculate the average flock velocity of neighbors within a given radius."""
        velocities = np.array(
            [
                np.array(
                    [
                        state.flock_twist.linear.x,
                        state.flock_twist.linear.y,
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
    
    def state_to_twist(self, state):
        """
        Convert the state to a Twist message.
        """
        twist = Twist()
        twist.linear.x = state[0]
        twist.linear.y = state[1]
        twist.angular.z = state[2]
        return twist

    def plot_results(self):
        """Plot the results of the control loop."""
        if not self.actual_positions:  # If no data was collected
            rospy.logwarn("No data to plot!")
            return

        rospy.loginfo(f"Plotting results for robot {self.tb_id}")
        rospy.loginfo(f"Number of data points: {len(self.actual_positions)}")

        # Convert lists to numpy arrays
        actual_positions = np.array(self.actual_positions)
        actual_velocities = np.array(self.actual_velocities)
        target_velocities = np.array(self.target_velocities)
        times = np.array(self.times)
    

        rospy.loginfo(f"Data shapes - positions: {actual_positions.shape}, velocities: {actual_velocities.shape}, times: {times.shape}")

        # Create figure with subplots
        fig, axs = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(f'Robot {self.tb_id} Control Results')

        # Position plot
        axs[0, 0].plot(actual_positions[:, 0], actual_positions[:, 1], 'b-', label='Actual')
        axs[0, 0].set_title('Position')
        axs[0, 0].set_xlabel('X (m)')
        axs[0, 0].set_ylabel('Y (m)')
        axs[0, 0].legend()
        axs[0, 0].grid(True)

        # Linear velocity plot
        axs[0, 1].plot(times, actual_velocities[:, 0], 'b-', label='Actual')
        axs[0, 1].plot(times, target_velocities[:, 0], 'r--', label='Target')
        axs[0, 1].set_title('Linear Velocity')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Velocity (m/s)')
        axs[0, 1].legend()
        axs[0, 1].grid(True)

        # Angular velocity plot
        axs[1, 0].plot(times, actual_velocities[:, 1], 'b-', label='Actual')
        axs[1, 0].plot(times, target_velocities[:, 1], 'r--', label='Target')
        axs[1, 0].set_title('Angular Velocity')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Velocity (rad/s)')
        axs[1, 0].legend()
        axs[1, 0].grid(True)

        # Velocity magnitude plot
        actual_vel_mag = np.linalg.norm(actual_velocities, axis=1)
        target_vel_mag = np.linalg.norm(target_velocities, axis=1)
        axs[1, 1].plot(times, actual_vel_mag, 'b-', label='Actual')
        axs[1, 1].plot(times, target_vel_mag, 'r--', label='Target')
        axs[1, 1].set_title('Velocity Magnitude')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Magnitude')
        axs[1, 1].legend()
        axs[1, 1].grid(True)

        # Adjust layout and save
        plt.tight_layout()
        
        # Create plots directory at the same level as src
        src_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        plots_dir = os.path.join(os.path.dirname(src_dir), 'plots')
        os.makedirs(plots_dir, exist_ok=True)
        
        # Save plot with absolute path
        save_path = os.path.join(plots_dir, f'robot_{self.tb_id}_control_results.png')
        plt.savefig(save_path)
        rospy.loginfo(f"Plot saved to {save_path}")
        plt.close()
