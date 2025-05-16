#!/usr/bin/env python3
import rospy
from swarm_explorer.msg import ExplorerStateMsg, ExplorerMapMsg
import numpy as np


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

        rospy.loginfo("Swarm relay node started")
        rospy.spin()

    def states_cb(self, msg: ExplorerStateMsg):
        """
        Update the position of the robot with the given id.
        The position is given in the message.
        """
        self.bots_dict[msg.robot_id] = msg  # TODO: any issues with pointer stuff?

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
            rospy.logwarn(f"Robot {msg.robot_id} not found in bots_dict")
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
                    topic, ExplorerMapMsg, queue_size=1
                )
            self.map_pubs[robot_id].publish(msg)

    def _within_radius(self, sender_id: int, recipient_id: int) -> bool:
        p1 = self.bots_dict.get(sender_id).pose.position
        p2 = self.bots_dict.get(recipient_id).pose.position
        if not p1 or not p2:
            return False
        dx, dy = p1.x - p2.x, p1.y - p2.y
        return (dx * dx + dy * dy) ** 0.5 <= self.comm_radius

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
