#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
import tf_conversions

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from final_proj.msg import ExplorerState, SLAMData
from final_proj.src.frontier import FrontierUpdater
from final_proj.src.controller import TurtlebotController
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

        # topics
        if self.map_type not in ["occupancy", "slam"]:
            rospy.logerr("Invalid map topic specified. Exiting.")
            rospy.signal_shutdown("Invalid map topic specified.")
            return
        self.map_topic: str = "/swarm/robot_maps"
        self.map_pub_topic: str = f"/robot_{self.bot_id}/incoming/map"
        self.state_topic: str = "/swarm/robot_states"
        self.state_pub_topic: str = f"/robot_{self.bot_id}/incoming/state"
        self.cmd_topic: str = f"/robot_{self.bot_id}/cmd_vel"

        # storage for callbacks
        self.latest_map: OccupancyGrid | SLAMData = None 
        self.neighbor_states: Dict[int, Odometry] = {}  # robot_id → Odometry

        # publish static map→odom
        br = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = f"robot_{self.bot_id}/odom"
        t.transform.translation.x = self.curr_pose["x"]
        t.transform.translation.y = self.curr_pose["y"]
        t.transform.translation.z = 0
        quat = tf_conversions.transformations.quaternion_from_euler(
            0, 0, self.curr_pose["theta"]  # roll, pitch, yaw
        )
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        br.sendTransform(t)

        # publishers and subscribers
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.pub_state = rospy.Publisher(
            self.state_pub_topic, ExplorerState, queue_size=1
        )
        self.pub_map = rospy.Publisher(self.map_pub_topic, rospy.AnyMsg, queue_size=1)
        # subscribe to other robots' states and maps
        rospy.Subscriber(self.state_topic, ExplorerState, self._state_callback)
        rospy.Subscriber(self.map_topic, rospy.AnyMsg, self._map_callback)
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
        
        Kp = np.diag([2.0, 0.8])
        Kd = np.diag([-0.5, 0.5])
        Ki = np.diag([0.0, 0.0])
        
        self.controller = TurtlebotController(
            tb_id=self.bot_id,
            cohesion_radius=self.cohesion_radius,
            separation_radius=self.separation_radius,
            alignment_radius=self.alignment_radius,
            collision_radius=self.collision_radius,
            Kp=Kp,
            Kd=Kd,
            Ki=Ki,
        )

    def initialize_params(self):
        """
        Initialize parameters for the ExplorerBot.
        This function is called in the constructor to set up the initial state.
        """
        self.bot_id: int = rospy.get_param("~robot_id")  # 1-based

        poses: List[Dict[str, float]] = rospy.get_param(
            "/initial_poses"
        )  # list of dicts
        self.curr_pose: Dict[str, float] = poses[self.bot_id - 1]
        self.curr_odom: Odometry = self._pose_to_odom(self.curr_pose)  # type: Odometry
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

        self.map_type: str = rospy.get_param("~map_type")  # either occupancy or slam


    def _map_callback(self, msg):
        """
        Callback function for the map topic.
        This function will be called whenever a new message is received on the map topic.
        """
        # You can add your processing logic here
        neighbor_map = msg  # Assuming msg.data is the map data you want to process
        self.latest_map = self.frontier_updater.update(self.latest_map, neighbor_map)

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
        odom.header.frame_id = "map"
        odom.child_frame_id = f"robot_{self.bot_id}/odom"
        odom.pose.pose.position.x = pose["x"]
        odom.pose.pose.position.y = pose["y"]
        odom.pose.pose.position.z = 0
        quat = tf_conversions.transformations.quaternion_from_euler(
            0, 0, pose["theta"]  # roll, pitch, yaw
        )
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
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
        self.curr_pose = {"x": x, "y": y, "theta": theta}
        self.curr_odom = msg

    def _get_current_pose(self):
        """Return the current pose of the robot."""
        # return pose if available
        if self.curr_pose is not None:
            return self.curr_pose

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


    def run(self):
        rate = rospy.Rate(10)  # 10 Hz control loop
        while not rospy.is_shutdown() or not self.frontier_updater.map_fully_known():
            # Process the latest map and neighbor states
            if self.latest_map is None:
                # Process the map data
                rate.sleep()
                continue
            # TODO: frontier_updater call

            # Call to the controller
            true_neighbor_states = {
                k: v
                for k, v in self.neighbor_states.items()
                if (rospy.Time.now() - v.header.stamp).to_sec() < self.max_neighbor_age
            }
            control_input: Twist = self.controller.step_control(
                curr_odom=self.curr_odom,  # current odom
                self.latest_map,  # latest map
                true_neighbor_states,  # neighbor states
                self.max_neighbor_age,
            )
            self.pub_cmd.publish(control_input)


            # TODO: Do we need to do any frontier/map updates in this loop?
            # May already be processed by the callback
            # TODO: finish the calls to controller
            state_msg = ExplorerState()
            state_msg.robot_id = self.bot_id
            state_msg.odometry = self._pose_to_odom(self._get_current_pose())
            # TODO: fill in flock velocity and frontier velocity from controller
            # I think this means we do need to call controller first
            state_msg.flock_twist = self.controller.flock_vel
            state_msg.frontier_twist = self.controller.frontier_vel
            self.pub_state.publish(state_msg)
            self.pub_map.publish(self.latest_map)
            rate.sleep()

            


if __name__ == "__main__":
    bot = ExplorerBot()
    bot.run()
