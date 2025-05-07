#!/usr/bin/env python
import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
import tf_conversions

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from final_proj.msg import ExplorerState, SLAMData
from final_proj.src.frontier import FrontierUpdater
from final_proj.src.controller import Controller


class ExplorerBot:
    def __init__(self):
        """
        Initializes the ExplorerBot with info from the parameter server.
        """
        rospy.init_node("explorer_bot")
        self.bot_id = rospy.get_param("~robot_id")  # 1-based

        poses = rospy.get_param("/initial_poses")  # list of dicts
        self.my_pose = poses[self.bot_id - 1]
        self.comm_radius = rospy.get_param("~comm_radius", 5.0)  # meters
        self.max_neighbor_age = rospy.get_param("~max_neighbor_age", 0.5)  # seconds

        # topics
        self.map_type = rospy.get_param("~map_type")  # either occupancy or slam
        if self.map_type not in ["occupancy", "slam"]:
            rospy.logerr("Invalid map topic specified. Exiting.")
            rospy.signal_shutdown("Invalid map topic specified.")
            return
        self.map_topic = f"/mapping/{self.map_type}"
        self.map_topic = "/swarm/robot_maps"
        self.map_pub_topic = f"/robot_{self.bot_id}/incoming/map"
        self.state_topic = "/swarm/robot_states"
        self.state_pub_topic = f"/robot_{self.bot_id}/incoming/state"
        self.cmd_topic = f"/robot_{self.bot_id}/cmd_vel"

        # storage for callbacks
        self.latest_map = None  # type: OccupancyGrid | SLAMData
        self.neighbor_states = dict()  # robot_id → Odometry

        # publish static map→odom
        br = StaticTransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = f"robot_{self.bot_id}/odom"
        t.transform.translation.x = self.my_pose["x"]
        t.transform.translation.y = self.my_pose["y"]
        t.transform.translation.z = 0
        quat = tf_conversions.transformations.quaternion_from_euler(
            0, 0, self.my_pose["theta"]  # roll, pitch, yaw
        )
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        br.sendTransform(t)

        # publishers and subscribers
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        # communicate states and maps to other robots
        self.pub_state = rospy.Publisher(
            self.state_pub_topic, ExplorerState, queue_size=1
        )
        self.pub_map = rospy.Publisher(self.map_pub_topic, rospy.AnyMsg, queue_size=1)
        # subscribe to other robots' states and maps
        rospy.Subscriber(self.statetopic, ExplorerState, self._state_callback)
        rospy.Subscriber(self.map_topic, rospy.AnyMsg, self._map_callback)

        # TODO: fill in these with the correct classes/parameters
        self.frontier_updater = FrontierUpdater()
        self.controller = Controller(
            neighbors=self.neighbor_states, comm_radius=self.comm_radius
        )

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
        self.neighbor_states[msg.robot_id] = msg.odometry

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz control loop
        while not rospy.is_shutdown() or not self.frontier_updater.map_fully_known():
            # Process the latest map and neighbor states
            if self.latest_map is None:
                # Process the map data
                rate.sleep()
                continue

            # TODO: Do we need to do any frontier/map updates in this loop?
            # May already be processed by the callback


if __name__ == "__main__":
    bot = ExplorerBot()
    bot.run()
