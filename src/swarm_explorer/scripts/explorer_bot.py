#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
import tf_conversions
import signal
import sys

from mapping.occupancy_grid_2d import OccupancyGrid2d
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from swarm_explorer.msg import ExplorerStateMsg, ExplorerMapMsg
from swarm_explorer.frontier_updater import FrontierUpdater
from swarm_explorer.controller import TurtlebotController
from swarm_explorer.env_config import EnvConfig
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
        
        self.map_topic: str = f"/robot_{self.bot_id}/incoming/map"
        self.map_pub_topic: str = "/swarm/robot_maps"
        self.state_topic: str = f"/robot_{self.bot_id}/incoming/state"
        self.state_pub_topic: str = "/swarm/robot_states"
        # self.cmd_topic: str = f"/robot_{self.bot_id}/cmd_vel"

        # storage for callbacks
        self.latest_map: OccupancyGrid2d = OccupancyGrid2d()
        self.latest_map.Initialize()
        self._merged_maps = 0
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

        # Load environment configurations from parameter server (done within the class)
        self.env_config = EnvConfig()
        
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
            env_config=self.env_config,
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
        self.comm_radius: float = rospy.get_param("/comm_radius")  # meters
        self.max_neighbor_age: float = rospy.get_param(
            "/max_neighbor_age"
        )  # seconds
        # Algorithm radii
        self.cohesion_radius: float = rospy.get_param("/cohesion_radius")  # meters
        self.separation_radius: float = rospy.get_param(
            "/separation_radius"
        )
        self.alignment_radius: float = rospy.get_param(
            "/alignment_radius"
        )
        self.collision_radius: float = rospy.get_param(
            "/collision_radius"
        )
        # Algorithm weights
        self.cohesion_weight: float = rospy.get_param(
            "/cohesion_weight"
        )
        self.separation_weight: float = rospy.get_param(
            "/separation_weight"
        )
        self.alignment_weight: float = rospy.get_param(
            "/alignment_weight"
        )
        self.obstacle_weight: float = rospy.get_param(
            "/obstacle_weight"
        )
        self.wall_weight: float = rospy.get_param("/wall_weight")
        self.frontier_weight: float = rospy.get_param(
            "/frontier_weight"
        )
        # Frontier parameters
        self.frontier_dist_wt: float = rospy.get_param(
            "/frontier_dist_weight"
        )
        self.frontier_size_wt: float = rospy.get_param(
            "/frontier_size_weight"
        )

        # self.map_type: str = rospy.get_param("~map_type")  # either occupancy or slam

    def _map_callback(self, msg: ExplorerMapMsg):
        """
        Callback function for the map topic.
        This function will be called whenever a new message is received on the map topic.
        """
        if msg.robot_id == self.bot_id or self.latest_map is None:
            return

        width = msg.grid.info.width
        height = msg.grid.info.height
        if width != self.latest_map._x_num or height != self.latest_map._y_num:
            rospy.logwarn_throttle(
                2.0,
                "Robot %d: map merge skip from robot %d (shape %dx%d != %dx%d).",
                self.bot_id,
                msg.robot_id,
                width,
                height,
                self.latest_map._x_num,
                self.latest_map._y_num,
            )
            return

        data = np.array(msg.grid.data, dtype=np.int16).reshape((width, height))
        known_mask = data >= 0  # -1 is unknown in nav_msgs/OccupancyGrid
        if not np.any(known_mask):
            return

        probs = np.clip(data[known_mask].astype(np.float64) / 100.0, 0.01, 0.99)
        incoming_log_odds = np.log(probs / (1.0 - probs))
        fusion_gain = 0.25
        self.latest_map._map[known_mask] = np.clip(
            self.latest_map._map[known_mask] + fusion_gain * incoming_log_odds,
            self.latest_map._free_threshold,
            self.latest_map._occupied_threshold,
        )

        self._merged_maps += 1
        rospy.loginfo_throttle(
            2.0, "Robot %d: merged neighbor maps=%d", self.bot_id, self._merged_maps
        )

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

    def _get_current_pose_map(self):
        """Return robot pose in the global `map` frame."""
        try:
            t = self.tfBuffer.lookup_transform(
                "map",  # target frame
                f"robot_{self.bot_id}/base_footprint",  # source frame
                rospy.Time(0),
                rospy.Duration(0.2),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            (_, _, theta) = tf_conversions.transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
            return (x, y, theta)
        except Exception:
            rospy.logwarn_throttle(
                2.0, "Robot %d: failed TF lookup for map-frame pose.", self.bot_id
            )
            return None

    @staticmethod
    def _map_pose_stamped_from_tuple(map_pose):
        """Build `map` frame PoseStamped from (x, y, theta)."""
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = map_pose[0]
        msg.pose.position.y = map_pose[1]
        msg.pose.position.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, map_pose[2])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        rospy.loginfo("Shutdown signal received. Plotting results and exiting...")
        self.should_exit = True
        self.controller.plot_results()
        rospy.signal_shutdown("User requested shutdown")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz control loop
        
        # Main loop
        while not rospy.is_shutdown() and not self.should_exit and not self.frontier_updater.map_fully_known():
            # Process the latest map and neighbor states
            if self.latest_map is None:
                # Process the map data
                rate.sleep()
                continue
            map_pose = self._get_current_pose_map()
            if map_pose is None:
                rate.sleep()
                continue

            # Update frontiers based on the latest map (map frame).
            self.frontier_updater.update_frontiers(
                np.array([map_pose[0], map_pose[1]])
            )
            # TODO: what message type is our map?
            map_msg = self.latest_map.to_msg()
            map_msg.robot_id = self.bot_id
            self.pub_map.publish(map_msg)

            # Find target frontier
            best_frontier = self.frontier_updater.get_best_frontier(
                np.array([map_pose[0], map_pose[1]])
            )
            rospy.loginfo_throttle(
                2.0,
                "Robot %d: frontiers=%d",
                self.bot_id,
                len(self.frontier_updater.frontiers),
            )
            if best_frontier is None:
                # Still publish state so relay can track this robot.
                state_msg = ExplorerStateMsg()
                state_msg.robot_id = self.bot_id
                state_msg.odometry = self.curr_odom
                state_msg.map_pose = self._map_pose_stamped_from_tuple(map_pose)
                state_msg.pose = state_msg.map_pose.pose
                self.pub_state.publish(state_msg)

                # Bootstrap exploration: slow spiral to discover free space/frontiers.
                search_cmd = Twist()
                search_cmd.linear.x = 0.04
                search_cmd.angular.z = 0.35
                self.controller.cmd(search_cmd)
                rospy.loginfo_throttle(
                    2.0, "Robot %d: no frontier yet, running spiral search.", self.bot_id
                )
                rate.sleep()
                continue

            # Call to the controller
            frontier_target = self.frontier_updater.frontier_to_world_point(best_frontier)
            true_neighbor_states = {
                k: v
                for k, v in self.neighbor_states.items()
                if (rospy.Time.now() - v.odometry.header.stamp).to_sec() < self.max_neighbor_age
            }
            ref_vel = self.controller.calc_reference_vels(
                curr_state=np.array([map_pose[0], map_pose[1], map_pose[2]]), # map-frame state
                latest_map=self.latest_map, # latest map
                neighbor_states=true_neighbor_states, # neighbor states
                frontier_target=frontier_target, # best frontier world target
            )
            self.controller.step_control(
                target_state=ref_vel,  # open loop input
                curr_odom=self.curr_odom,  # current odom
            )

            state_msg = ExplorerStateMsg()
            state_msg.robot_id = self.bot_id
            state_msg.odometry = self.curr_odom
            state_msg.map_pose = self._map_pose_stamped_from_tuple(map_pose)
            state_msg.pose = state_msg.map_pose.pose
            flock_twist = Twist()
            flock_twist.linear.x = self.controller.flock_vel[0]
            flock_twist.linear.y = self.controller.flock_vel[1]
            frontier_twist = Twist()
            frontier_twist.linear.x = self.controller.frontier_vel[0]
            frontier_twist.linear.y = self.controller.frontier_vel[1]
            state_msg.flock_twist = flock_twist
            state_msg.frontier_twist = frontier_twist
            self.pub_state.publish(state_msg)

            rate.sleep()

        # If we exit the loop normally (map fully known), plot results
        if self.frontier_updater.map_fully_known():
            rospy.loginfo("Map fully explored! Plotting results...")
            self.controller.plot_results()


if __name__ == "__main__":
    bot = ExplorerBot()
    bot.run()