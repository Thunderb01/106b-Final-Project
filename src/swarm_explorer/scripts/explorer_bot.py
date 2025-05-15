#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
import tf_conversions
import signal
import sys

from mapping.occupancy_grid_2d import OccupancyGrid2d
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from swarm_explorer.msg import ExplorerStateMsg, ExplorerMapMsg
from swarm_explorer.frontier_updater import FrontierUpdater
from swarm_explorer.controller import TurtlebotController
from typing import Dict, List


class ExplorerBot:
    """
    Class for the ExplorerBot that handles communication and state updates.
    """

    def __init__(self):
        """
        Initializes the ExplorerBot with info from the parameter server.
        """
        rospy.init_node("explorer_bot")
        # Initialize parameters from parameter server
        self.initialize_params()

        # topics
        # if self.map_type not in ["occupancy", "slam"]:
        #     rospy.logerr("Invalid map topic specified. Exiting.")
        #     rospy.signal_shutdown("Invalid map topic specified.")
        #     return
        self.map_topic: str = "/swarm/robot_maps"
        self.map_pub_topic: str = f"/robot_{self.bot_id}/incoming/map"
        self.state_topic: str = "/swarm/robot_states"
        self.state_pub_topic: str = f"/robot_{self.bot_id}/incoming/state"
        # self.cmd_topic: str = f"/robot_{self.bot_id}/cmd_vel"

        # storage for callbacks
        self.latest_map: OccupancyGrid2d = OccupancyGrid2d()
        self.latest_map.Initialize()
        self.neighbor_states: Dict[int, Odometry] = {}  # robot_id → Odometry

        # publishers and subscribers
        # self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.pub_state = rospy.Publisher(
            self.state_pub_topic, ExplorerStateMsg, queue_size=1
        )
        self.pub_map = rospy.Publisher(self.map_pub_topic, ExplorerMapMsg, queue_size=1)
        # subscribe to other robots' states and maps
        rospy.Subscriber(self.state_topic, ExplorerStateMsg, self._state_callback)
        rospy.Subscriber(self.map_topic, ExplorerMapMsg, self._map_callback)
        rospy.Subscriber(f"/robot_{self.bot_id}/odom/", Odometry, self._odom_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # TODO: fill in these with the correct classes/parameters
        self.frontier_updater = FrontierUpdater(
            robot_id=self.bot_id,
            occupancy_map=self.latest_map,
            frontier_dist_wt=self.frontier_dist_wt,
            frontier_size_wt=self.frontier_size_wt,
        )
        
        Kp = np.diag([2.0, 0.4])
        Kd = np.diag([-0.5, 0.2])
        Ki = np.diag([0.0, 0.0])

        self.controller = TurtlebotController(
            tb_id=self.bot_id,
            # map_obj=self.latest_map,
            # neighbor_states=self.neighbor_states,
            cohesion_radius=self.cohesion_radius,
            separation_radius=self.separation_radius,
            alignment_radius=self.alignment_radius,
            collision_radius=self.collision_radius,
            cohesion_weight=self.cohesion_weight,
            separation_weight=self.separation_weight,
            alignment_weight=self.alignment_weight,
            obstacle_weight=self.obstacle_weight,
            wall_weight=self.wall_weight,
            frontier_weight=self.frontier_weight,
            Kp=Kp,
            Kd=Kd,
            Ki=Ki,
        )

        # Set up signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        self.should_exit = False

    def initialize_params(self):
        """
        Initialize parameters for the ExplorerBot.
        This function is called in the constructor to set up the initial state.
        """
        self.bot_id: int = rospy.get_param("~robot_id")  # 1-based

        poses: List[Dict[str, float]] = rospy.get_param(
            "/initial_poses"
        )  # list of dicts
        self.curr_state: Dict[str, float] = poses[self.bot_id - 1]
        self.curr_vel: Dict[str, float] = {"x_dot": 0, "y_dot": 0}
        self.curr_odom: Odometry = self._pose_to_odom(self.curr_state)  # type: Odometry
        # Communication parameters
        self.comm_radius: float = rospy.get_param("~comm_radius", 5.0)  # meters
        self.max_neighbor_age: float = rospy.get_param(
            "~max_neighbor_age", 0.5
        )  # seconds
        # Algorithm radii
        self.cohesion_radius: float = rospy.get_param("~cohesion_radius", 1.0)  # meters
        self.separation_radius: float = rospy.get_param(
            "~separation_radius", 0.5
        )
        self.alignment_radius: float = rospy.get_param(
            "~alignment_radius", 1.0
        )
        self.collision_radius: float = rospy.get_param(
            "~collision_radius", 0.5
        )
        # Algorithm weights
        self.cohesion_weight: float = rospy.get_param(
            "~cohesion_weight", 0.23
        )
        self.separation_weight: float = rospy.get_param(
            "~separation_weight", 1.1
        )
        self.alignment_weight: float = rospy.get_param(
            "~alignment_weight", 0.5
        )
        self.obstacle_weight: float = rospy.get_param(
            "~obstacle_weight", 1.1
        )
        self.wall_weight: float = rospy.get_param("~wall_weight", 1.1)
        self.frontier_weight: float = rospy.get_param(
            "~frontier_weight", 0.08
        )
        # Frontier parameters
        self.frontier_dist_wt: float = rospy.get_param(
            "~frontier_dist_wt", 0.001
        )
        self.frontier_size_wt: float = rospy.get_param(
            "~frontier_size_wt", 1.0
        )

        # self.map_type: str = rospy.get_param("~map_type")  # either occupancy or slam

    def _map_callback(self, msg: ExplorerMapMsg):
        """
        Callback function for the map topic.
        This function will be called whenever a new message is received on the map topic.
        """
        # You can add your processing logic here
        neighbor_map = OccupancyGrid2d.from_msg(msg)
        self.latest_map._map = OccupancyGrid2d.merge_maps(self.latest_map, neighbor_map)

    def _state_callback(self, msg):
        """
        Callback function for the state topic.
        This function will be called whenever a new message is received on the state topic.
        """
        # Update the neighbor states
        self.neighbor_states[msg.robot_id] = msg

    def _pose_to_odom(self, pose):
        """
        Convert a pose to an odometry message.
        """
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = f"robot_{self.bot_id}/odom"
        odom.child_frame_id = f"robot_{self.bot_id}/base_footprint"
        odom.pose.pose.position.x = pose["x"]
        odom.pose.pose.position.y = pose["y"]
        odom.pose.pose.position.z = 0
        quat = tf_conversions.transformations.quaternion_from_euler(
            0, 0, pose["theta"]  # roll, pitch, yaw
        )
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        return odom

    def _odom_callback(self, msg: Odometry):
        """
        Update the odometry message with the latest pose.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = tf_conversions.transformations.euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )[2]
        self.curr_state = {"x": x, "y": y, "theta": theta}
        self.curr_odom = msg

    def _get_current_pose(self):
        """Return the current pose of the robot."""
        # return pose if available
        if self.curr_state is not None:
            return self.curr_state

        # or fall back to TF lookup:
        try:
            t = self.tfBuffer.lookup_transform(
                f"robot_{self.bot_id}/odom",  # target frame
                f"robot_{self.bot_id}/base_link",  # source frame
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            (_, _, theta) = tf_conversions.euler_from_quaternion([q.x, q.y, q.z, q.w])
            return (x, y, theta)
        except Exception:
            rospy.logerr("Failed to get current pose from TF")
            return None

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        rospy.loginfo("Shutdown signal received. Plotting results and exiting...")
        self.should_exit = True
        self.controller.plot_results()
        rospy.signal_shutdown("User requested shutdown")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz control loop
        # Plotting variables
        actual_positions = []
        actual_velocities = []
        target_positions = []
        target_velocities = []
        times = []
        # Main loop
        while not rospy.is_shutdown() and not self.should_exit and not self.frontier_updater.map_fully_known():
            # Process the latest map and neighbor states
            if self.latest_map is None:
                # Process the map data
                rate.sleep()
                continue
            # Update frontiers based on the latest map
            self.frontier_updater.update_frontiers(
                np.array([self.curr_state['x'], self.curr_state['y']])
            )
            # TODO: what message type is our map?
            map_msg = self.latest_map.to_msg()
            map_msg.robot_id = self.bot_id
            self.pub_map.publish(map_msg)

            # Find target frontier
            best_frontier = self.frontier_updater.get_best_frontier(
                np.array([self.curr_state["x"], self.curr_state["y"]])
            )
            if best_frontier is None:
                continue

            # Call to the controller
            true_neighbor_states = {
                k: v
                for k, v in self.neighbor_states.items()
                if (rospy.Time.now() - v.header.stamp).to_sec() < self.max_neighbor_age
            }
            ref_vel = self.controller.calc_reference_vels(
                curr_state=np.array([self.curr_state['x'], self.curr_state['y'], self.curr_state['theta']]), # current state
                latest_map=self.latest_map, # latest map
                neighbor_states=true_neighbor_states, # neighbor states
                best_frontier=best_frontier, # best frontier
            )
            self.controller.step_control(
                target_state=ref_vel,  # open loop input
                curr_odom=self.curr_odom,  # current odom
            )

            state_msg = ExplorerStateMsg()
            state_msg.robot_id = self.bot_id
            state_msg.odometry = self.curr_odom
            # TODO: fill in flock velocity and frontier velocity from controller
            # I think this means we do need to call controller first
            state_msg.flock_twist = self.controller.flock_vel
            state_msg.frontier_twist = self.controller.frontier_vel
            self.pub_state.publish(state_msg)

            rate.sleep()

        # If we exit the loop normally (map fully known), plot results
        if self.frontier_updater.map_fully_known():
            rospy.loginfo("Map fully explored! Plotting results...")
            self.controller.plot_results()


if __name__ == "__main__":
    bot = ExplorerBot()
    bot.run()