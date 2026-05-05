#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
import tf_conversions
import signal
import sys
from collections import deque
from visualization_msgs.msg import Marker

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
    MODE_SEARCH = "SEARCH"
    MODE_FRONTIER = "FRONTIER"
    MODE_ESCAPE = "ESCAPE"

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
        self.frontier_marker_topic = f"/robot_{self.bot_id}/vis/frontier_target"
        self.pub_frontier_marker = rospy.Publisher(
            self.frontier_marker_topic, Marker, queue_size=1
        )
        # subscribe to other robots' states and maps
        rospy.Subscriber(self.state_topic, ExplorerStateMsg, self._state_callback)
        rospy.Subscriber(self.map_topic, ExplorerMapMsg, self._map_callback)
        rospy.Subscriber(f"/robot_{self.bot_id}/odom/", Odometry, self._odom_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.frontier_hold_seconds = rospy.get_param("~frontier_hold_seconds", 4.0)
        self.frontier_reached_radius = rospy.get_param("~frontier_reached_radius", 0.6)
        self._held_frontier_target = None
        self._held_frontier_until = rospy.Time(0)
        self._pose_history = deque()
        self._escape_until = rospy.Time(0)
        self._escape_turn_sign = 1.0 if (self.bot_id % 2 == 0) else -1.0
        self.stuck_window_sec = rospy.get_param("~stuck_window_sec", 12.0)
        self.stuck_min_progress_m = rospy.get_param("~stuck_min_progress_m", 0.10)
        self.escape_duration_sec = rospy.get_param("~escape_duration_sec", 1.4)
        self.escape_cooldown_sec = rospy.get_param("~escape_cooldown_sec", 6.0)
        self.escape_reverse_x = rospy.get_param("~escape_reverse_x", -0.06)
        self.escape_turn_z = rospy.get_param("~escape_turn_z", 0.9)
        self.escape_trigger_obstacle_dist = rospy.get_param(
            "~escape_trigger_obstacle_dist", 0.35
        )
        self.escape_repeat_window_sec = rospy.get_param("~escape_repeat_window_sec", 25.0)
        self.escape_repeat_radius_m = rospy.get_param("~escape_repeat_radius_m", 0.9)
        self.escape_retarget_pause_sec = rospy.get_param("~escape_retarget_pause_sec", 2.5)
        self._escape_cooldown_until = rospy.Time(0)
        self._escape_history = deque()
        self._escape_cmd_linear_x = self.escape_reverse_x
        self._escape_cmd_angular_z = self.escape_turn_z
        self._frontier_pause_until = rospy.Time(0)
        self._mode = self.MODE_SEARCH
        self.log_frontier_counts = rospy.get_param("~log_frontier_counts", False)
        self._last_merge_time = rospy.Time(0)
        self.map_publish_hz = rospy.get_param("~map_publish_hz", 1.0)
        if self.map_publish_hz <= 0.0:
            self.map_publish_hz = 1.0
        self._last_map_pub_time = rospy.Time(0)
        self.map_fusion_gain = rospy.get_param("~map_fusion_gain", 0.20)
        self.map_fusion_confidence_min = rospy.get_param(
            "~map_fusion_confidence_min", 0.15
        )

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
        rospy.loginfo_throttle(
            2.0,
            "Robot %d: received map msg from robot_%d",
            self.bot_id,
            msg.robot_id,
        )
        if msg.robot_id == self.bot_id or self.latest_map is None:
            return

        width = msg.grid.info.width
        height = msg.grid.info.height
        if width <= 0 or height <= 0:
            return
        if len(msg.grid.data) != (width * height):
            rospy.logwarn_throttle(
                2.0,
                "Robot %d: map merge skip from robot %d (bad data length).",
                self.bot_id,
                msg.robot_id,
            )
            return

        data = np.array(msg.grid.data, dtype=np.int16).reshape((width, height))
        known_mask = data >= 0  # -1 is unknown in nav_msgs/OccupancyGrid
        if not np.any(known_mask):
            rospy.loginfo_throttle(
                2.0,
                "Robot %d: map msg from robot_%d had no known cells.",
                self.bot_id,
                msg.robot_id,
            )
            return

        local_map = self.latest_map._map
        local_known_before = int(
            np.sum(
                (local_map < self.latest_map._free_threshold)
                | (local_map > self.latest_map._occupied_threshold)
            )
        )

        probs = np.clip(data[known_mask].astype(np.float64) / 100.0, 0.01, 0.99)
        # Confidence is low near p=0.5 (uncertain) and high near 0/1.
        confidence = 2.0 * np.abs(probs - 0.5)  # [0, 1]
        confidence_mask = confidence >= self.map_fusion_confidence_min
        if not np.any(confidence_mask):
            rospy.loginfo_throttle(
                2.0,
                "Robot %d: map msg from robot_%d below fusion confidence threshold.",
                self.bot_id,
                msg.robot_id,
            )
            return
        incoming_log_odds = np.log(probs[confidence_mask] / (1.0 - probs[confidence_mask]))
        scaled_gain = self.map_fusion_gain * confidence[confidence_mask]

        known_indices = np.argwhere(known_mask)
        confident_indices = known_indices[confidence_mask]

        in_res = float(msg.grid.info.resolution)
        if in_res <= 0.0:
            rospy.logwarn_throttle(
                2.0,
                "Robot %d: map merge skip from robot %d (non-positive resolution).",
                self.bot_id,
                msg.robot_id,
            )
            return
        in_origin_x = float(msg.grid.info.origin.position.x)
        in_origin_y = float(msg.grid.info.origin.position.y)
        in_frame = msg.grid.header.frame_id if msg.grid.header.frame_id else "map"
        local_frame = self.latest_map._fixed_frame

        # Convert incoming confident cells to world/map-frame coordinates.
        in_x = in_origin_x + (confident_indices[:, 0].astype(np.float64) + 0.5) * in_res
        in_y = in_origin_y + (confident_indices[:, 1].astype(np.float64) + 0.5) * in_res

        if in_frame != local_frame:
            try:
                tf_msg = self.tfBuffer.lookup_transform(
                    local_frame,
                    in_frame,
                    rospy.Time(0),
                    rospy.Duration(0.2),
                )
            except Exception:
                rospy.logwarn_throttle(
                    2.0,
                    "Robot %d: map merge skip from robot %d (no TF %s -> %s).",
                    self.bot_id,
                    msg.robot_id,
                    in_frame,
                    local_frame,
                )
                return

            tx = tf_msg.transform.translation.x
            ty = tf_msg.transform.translation.y
            q = tf_msg.transform.rotation
            (_, _, yaw) = tf_conversions.transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
            c = np.cos(yaw)
            s = np.sin(yaw)
            x_local = c * in_x - s * in_y + tx
            y_local = s * in_x + c * in_y + ty
        else:
            x_local = in_x
            y_local = in_y

        # Project into local map indices.
        ii = np.floor((x_local - self.latest_map._x_min) / self.latest_map._x_res).astype(np.int32)
        jj = np.floor((y_local - self.latest_map._y_min) / self.latest_map._y_res).astype(np.int32)
        valid = (
            (ii >= 0)
            & (ii < self.latest_map._x_num)
            & (jj >= 0)
            & (jj < self.latest_map._y_num)
        )
        if not np.any(valid):
            rospy.loginfo_throttle(
                2.0,
                "Robot %d: map msg from robot_%d had no overlapping confident cells.",
                self.bot_id,
                msg.robot_id,
            )
            return

        ii = ii[valid]
        jj = jj[valid]
        delta = scaled_gain[valid] * incoming_log_odds[valid]
        # Keep merged values slightly outside unknown bounds so frontier/known
        # checks classify them as known instead of "exactly threshold == unknown".
        eps = 1e-3
        np.add.at(local_map, (ii, jj), delta)
        local_map[:, :] = np.clip(
            local_map,
            self.latest_map._free_threshold - eps,
            self.latest_map._occupied_threshold + eps,
        )
        local_known_after = int(
            np.sum(
                (local_map < self.latest_map._free_threshold)
                | (local_map > self.latest_map._occupied_threshold)
            )
        )
        known_gain = local_known_after - local_known_before

        self._merged_maps += 1
        self._last_merge_time = rospy.Time.now()
        rospy.loginfo_throttle(
            2.0,
            "Robot %d: merged maps=%d (last from robot_%d, known +%d)",
            self.bot_id,
            self._merged_maps,
            msg.robot_id,
            known_gain,
        )

    def _start_escape(self, reason, map_pose):
        now = rospy.Time.now()
        self._escape_history.append((now, map_pose[0], map_pose[1]))
        history_cutoff = now - rospy.Duration(self.escape_repeat_window_sec)
        while self._escape_history and self._escape_history[0][0] < history_cutoff:
            self._escape_history.popleft()

        nearby_repeat_count = 0
        for (_, hx, hy) in self._escape_history:
            if np.hypot(map_pose[0] - hx, map_pose[1] - hy) <= self.escape_repeat_radius_m:
                nearby_repeat_count += 1

        repeat_scale = min(1.0 + 0.35 * max(nearby_repeat_count - 1, 0), 2.0)
        self._escape_cmd_linear_x = self.escape_reverse_x * repeat_scale
        self._escape_cmd_angular_z = self.escape_turn_z * repeat_scale
        self._escape_until = now + rospy.Duration(self.escape_duration_sec * repeat_scale)
        self._escape_cooldown_until = now + rospy.Duration(self.escape_cooldown_sec)
        self._escape_turn_sign *= -1.0
        self._set_mode(self.MODE_ESCAPE, reason)
        self._held_frontier_target = None
        self._held_frontier_until = rospy.Time(0)
        self._frontier_pause_until = now + rospy.Duration(self.escape_retarget_pause_sec)
        self._pose_history.clear()
        
    def _set_mode(self, new_mode, reason):
        if self._mode == new_mode:
            return
        rospy.logwarn("Robot %d: mode %s -> %s (%s)", self.bot_id, self._mode, new_mode, reason)
        self._mode = new_mode

    def _nearest_obstacle_distance(self, map_pose):
        obstacles = self.latest_map.get_surrounding_obstacles(
            np.array([map_pose[0], map_pose[1]]),
            radius=max(self.collision_radius, 1.0),
            is_point=True,
        )
        if not obstacles:
            return float("inf")
        return float(obstacles[0][1])

    def _update_stuck_state(self, map_pose):
        """
        Trigger escape when nearly stationary next to mapped obstacles.
        Must run even without an active frontier (e.g. wedged in a corner with
        no frontier target); previously we gated on has_frontier and robot_3
        could sit forever without escape logs.
        """
        now = rospy.Time.now()
        self._pose_history.append((now, map_pose[0], map_pose[1]))
        window_start = now - rospy.Duration(self.stuck_window_sec)
        while self._pose_history and self._pose_history[0][0] < window_start:
            self._pose_history.popleft()

        if len(self._pose_history) < 2:
            return

        first = self._pose_history[0]
        last = self._pose_history[-1]
        progress = float(np.hypot(last[1] - first[1], last[2] - first[2]))
        nearest_obstacle_dist = self._nearest_obstacle_distance(map_pose)
        near_obstacle = nearest_obstacle_dist <= self.escape_trigger_obstacle_dist
        if progress < self.stuck_min_progress_m and near_obstacle:
            if now < self._escape_cooldown_until or now < self._escape_until:
                rospy.loginfo_throttle(
                    3.0,
                    "Robot %d: stuck-like (progress=%.2fm, obstacle=%.2fm) but escape "
                    "cooldown/timer active.",
                    self.bot_id,
                    progress,
                    nearest_obstacle_dist,
                )
                return
            self._start_escape(
                "low progress %.2fm in %.1fs near obstacle (%.2fm)"
                % (progress, self.stuck_window_sec, nearest_obstacle_dist),
                map_pose,
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

    def _publish_frontier_target_marker(self, frontier_target):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "frontier_target"
        marker.id = int(self.bot_id)
        marker.pose.orientation.w = 1.0

        if frontier_target is None:
            marker.action = Marker.DELETE
            self.pub_frontier_marker.publish(marker)
            return

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(frontier_target[0])
        marker.pose.position.y = float(frontier_target[1])
        marker.pose.position.z = 0.06
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.95
        self.pub_frontier_marker.publish(marker)

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
            now = rospy.Time.now()
            if (
                self._last_map_pub_time == rospy.Time(0)
                or (now - self._last_map_pub_time).to_sec() >= (1.0 / self.map_publish_hz)
            ):
                map_msg = self.latest_map.to_msg()
                map_msg.robot_id = self.bot_id
                self.pub_map.publish(map_msg)
                self._last_map_pub_time = now

            # Find target frontier
            best_frontier = self.frontier_updater.get_best_frontier(
                np.array([map_pose[0], map_pose[1]])
            )
            candidate_target = self.frontier_updater.frontier_to_world_point(
                best_frontier, reference_world_point=(map_pose[0], map_pose[1])
            )
            now = rospy.Time.now()

            # Frontier persistence: hold a selected target briefly to avoid
            # target thrashing that causes local spiraling.
            if self._held_frontier_target is not None:
                held_dist = np.linalg.norm(
                    np.array([map_pose[0], map_pose[1]]) - np.array(self._held_frontier_target)
                )
                if held_dist < self.frontier_reached_radius or now >= self._held_frontier_until:
                    self._held_frontier_target = None

            if self._held_frontier_target is None and candidate_target is not None:
                self._held_frontier_target = candidate_target
                self._held_frontier_until = now + rospy.Duration(self.frontier_hold_seconds)

            if now < self._frontier_pause_until:
                frontier_target = None
            else:
                frontier_target = self._held_frontier_target
            # Keep marker visible for debugging even during temporary retarget pause.
            marker_target = (
                self._held_frontier_target
                if self._held_frontier_target is not None
                else frontier_target
            )
            self._publish_frontier_target_marker(marker_target)
            self._update_stuck_state(map_pose)
            neighbor_count = len(
                {
                    k: v
                    for k, v in self.neighbor_states.items()
                    if (rospy.Time.now() - v.odometry.header.stamp).to_sec() < self.max_neighbor_age
                }
            )
            if neighbor_count > 0:
                since_last_merge = (rospy.Time.now() - self._last_merge_time).to_sec()
                if self._last_merge_time == rospy.Time(0) or since_last_merge > 8.0:
                    rospy.logwarn_throttle(
                        3.0,
                        "Robot %d: %d neighbors but no recent map merges (last %.1fs ago).",
                        self.bot_id,
                        neighbor_count,
                        since_last_merge if self._last_merge_time != rospy.Time(0) else -1.0,
                    )
            if self.log_frontier_counts:
                rospy.loginfo_throttle(
                    2.0,
                    "Robot %d: frontiers=%d",
                    self.bot_id,
                    len(self.frontier_updater.frontiers),
                )
            now = rospy.Time.now()
            if now < self._escape_until:
                self._set_mode(self.MODE_ESCAPE, "escape timer active")
                escape_cmd = Twist()
                escape_cmd.linear.x = self._escape_cmd_linear_x
                escape_cmd.angular.z = self._escape_turn_sign * self._escape_cmd_angular_z
                self.controller.cmd(escape_cmd)
                rospy.loginfo_throttle(1.0, "Robot %d: ESCAPE mode active.", self.bot_id)
                rate.sleep()
                continue
            elif self._mode == self.MODE_ESCAPE:
                self._set_mode(self.MODE_FRONTIER, "escape complete")

            if frontier_target is None:
                self._set_mode(self.MODE_SEARCH, "no frontier target")
                self._publish_frontier_target_marker(None)
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

            self._set_mode(self.MODE_FRONTIER, "frontier target selected")

            # Call to the controller
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