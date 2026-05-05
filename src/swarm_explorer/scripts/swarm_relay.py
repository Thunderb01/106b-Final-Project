#!/usr/bin/env python3
import rospy
from swarm_explorer.msg import ExplorerStateMsg, ExplorerMapMsg


class SwarmRelay(object):
    """
    Relay node for the swarm.
    This node receives messages from the robots and relays them to other robots
    within the communication radius.
    """

    def __init__(self):
        rospy.init_node("swarm_relay")
        self.comm_radius = rospy.get_param("~comm_radius", 5.0)  # meters
        # self.map_type = rospy.get_param("~map_type")
        self.bots_dict = {}  # robot_id → (x,y)

        # update positions, publish to other robots
        rospy.Subscriber("/swarm/robot_states", ExplorerStateMsg, self.states_cb)

        # update maps
        rospy.Subscriber("/swarm/robot_maps", ExplorerMapMsg, self.mapping_cb)

        # will hold publishers per robot
        self.state_pubs = {}
        self.map_pubs = {}
        self._active_links = set()
        self._relayed_map_msgs = 0

        rospy.loginfo("Swarm relay node started")
        rospy.spin()

    def states_cb(self, msg: ExplorerStateMsg):
        """
        Update the position of the robot with the given id.
        The position is given in the message.
        """
        self.bots_dict[msg.robot_id] = msg  # TODO: any issues with pointer stuff?

        # Track communication-link transitions for observability.
        for robot_id in list(self.bots_dict):
            if robot_id == msg.robot_id:
                continue
            pair = tuple(sorted((msg.robot_id, robot_id)))
            in_range = self._within_radius(sender_id=msg.robot_id, recipient_id=robot_id)
            if in_range and pair not in self._active_links:
                self._active_links.add(pair)
                rospy.loginfo(
                    "Comm link established between robot_%d and robot_%d", pair[0], pair[1]
                )
            elif not in_range and pair in self._active_links:
                self._active_links.remove(pair)
                rospy.loginfo(
                    "Comm link lost between robot_%d and robot_%d", pair[0], pair[1]
                )

        # publish the message to all robots within the communication radius
        for robot_id in list(self.bots_dict):
            if robot_id == msg.robot_id:
                continue
            if not self._within_radius(sender_id=msg.robot_id, recipient_id=robot_id):
                continue

            # lazy‐create publisher
            if robot_id not in self.state_pubs:
                topic = f"/robot_{robot_id}/incoming/state"  # TODO: change topic name
                self.state_pubs[robot_id] = rospy.Publisher(
                    topic, ExplorerStateMsg, queue_size=1
                )
            self.state_pubs[robot_id].publish(msg)

    def mapping_cb(self, msg: ExplorerMapMsg):
        """
        Update the map of the robot with the given id.
        The map is given in the message.
        """
        if msg.robot_id not in self.bots_dict:
            rospy.logwarn("Robot %d not found in bots_dict", msg.robot_id)
            return
        
        # publish the message to all robots within the communication radius
        for robot_id in list(self.bots_dict):
            if robot_id == msg.robot_id:
                continue
            if not self._within_radius(sender_id=msg.robot_id, recipient_id=robot_id):
                continue

            # lazy‐create publisher
            if robot_id not in self.map_pubs:
                topic = f"/robot_{robot_id}/incoming/map"  # TODO: change topic name
                self.map_pubs[robot_id] = rospy.Publisher(
                    topic, ExplorerMapMsg, queue_size=5
                )
            self.map_pubs[robot_id].publish(msg)
            self._relayed_map_msgs += 1
        rospy.loginfo_throttle(
            2.0,
            "Relay: map msgs relayed=%d (last from robot_%d)",
            self._relayed_map_msgs,
            msg.robot_id,
        )

    def _within_radius(self, sender_id: int, recipient_id: int) -> bool:
        sender_msg = self.bots_dict.get(sender_id)
        recipient_msg = self.bots_dict.get(recipient_id)
        if sender_msg is None or recipient_msg is None:
            return False
        p1x, p1y = self._map_pose_xy(sender_msg)
        p2x, p2y = self._map_pose_xy(recipient_msg)
        dx, dy = p1x - p2x, p1y - p2y
        return (dx * dx + dy * dy) ** 0.5 <= self.comm_radius

    @staticmethod
    def _map_pose_xy(msg: ExplorerStateMsg):
        """Planar position in a common frame; prefer map_pose, else odometry (legacy)."""
        if msg.map_pose.header.frame_id:
            p = msg.map_pose.pose.position
            return p.x, p.y
        p = msg.odometry.pose.pose.position
        return p.x, p.y

    def shutdown(self):
        """
        Shutdown the node.
        """
        rospy.loginfo("Shutting down swarm relay node")
        for pub in self.state_pubs.values():
            pub.unregister()
        for pub in self.map_pubs.values():
            pub.unregister()
        rospy.signal_shutdown("Swarm relay node shut down")


if __name__ == "__main__":
    SwarmRelay()
