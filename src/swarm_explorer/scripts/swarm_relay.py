#!/usr/bin/env python
import rospy
from msg import ExplorerState
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
        self.neighbors = {}  # robot_id → (x,y)

        # update positions, publish to other robots
        rospy.Subscriber("/swarm/heartbeat", ExplorerState, self.heartbeat_cb)

        # will hold publishers per robot
        self.pubs = {}

        rospy.loginfo("Swarm relay node started")
        rospy.spin()

    def heartbeat_cb(self, msg: ExplorerState):
        """
        Update the position of the robot with the given id.
        The position is given in the message.
        """
        self.neighbors[msg.robot_id] = msg  # TODO: any issues with pointer stuff?

        # publish the message to all robots within the communication radius
        for robot_id in self.neighbors:
            if robot_id == msg.robot_id:
                continue
            if not self._within_radius(msg.robot_id, robot_id):
                continue

            # lazy‐create publisher
            if robot_id not in self.pubs:
                topic = f"/robot_{robot_id}/incoming/heartbeat"
                self.pubs[robot_id] = rospy.Publisher(
                    topic, ExplorerState, queue_size=5
                )
            self.pubs[robot_id].publish(msg)

    def _within_radius(self, sender_id: str, recipient_id: str) -> bool:
        p1 = self.neighbors.get(sender_id).state
        p2 = self.neighbors.get(recipient_id).state
        if not p1 or not p2:
            return False
        dx, dy = p1.x - p2.x, p1.y - p2.y
        return (dx * dx + dy * dy) ** 0.5 <= self.comm_radius

    def shutdown(self):
        """
        Shutdown the node.
        """
        rospy.loginfo("Shutting down swarm relay node")
        for pub in self.pubs.values():
            pub.unregister()
        rospy.signal_shutdown("Swarm relay node shut down")


if __name__ == "__main__":
    SwarmRelay()
