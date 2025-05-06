#!/usr/bin/env python
import sys
import numpy as np
import rospy
import tf2_ros

from geometry_msgs.msg import Twist
from swarm_explorer.msg import BicycleCommandMsg, BicycleStateMsg, BicycleStateListMsg


class BicycleModelController(object):
    def __init__(self, tb_id, Kp):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher("/bicycle/cmd_vel", BicycleCommandMsg, queue_size=10)
        # TODO: have to change the topic this subscribes to
        self.sub = rospy.Subscriber(
            "/bicycle/state", BicycleStateListMsg, self.subscribe
        )
        self.state = BicycleStateMsg()
        self.tb_id = tb_id
        self.Kp = Kp
        self.length = 0.3
        self.neighbors = dict()
        rospy.on_shutdown(self.shutdown)

    def step_control(self, target_position, open_loop_input):
        """Specify a control law. For the grad/EC portion, you may want
        to edit this part to write your own closed loop controller.
        Note that this class constantly subscribes to the state of the robot,
        so the current configuratin of the robot is always stored in the
        variable self.state. You can use this as your state measurement
        when writing your closed loop controller.

        Parameters
        ----------
            target_position : target position at the current step in
                              [x, y, theta, phi] configuration space.
            open_loop_input : the prescribed open loop input at the current
                              step, as a [u1, u2] pair.
        Returns:
            None. It simply sends the computed command to the robot.
        """

        # self.cmd(open_loop_input)
        # q = np.array([self.state.x, self.state.y, self.state.theta, self.state.phi])
        q = self.state

        # Bicycle Model
        u = open_loop_input
        x_dot = np.cos(q[2]) * u[0]
        y_dot = np.sin(q[2]) * u[0]
        theta_dot = np.tan(q[3]) * u[0] / self.length
        phi_dot = u[1]
        q_next = np.vstack([x_dot, y_dot, theta_dot, phi_dot])

        # Position Error
        error = target_position.reshape(-1, 1) - q_next

        # PD Controller
        P = self.Kp @ error
        # No D term

        control_input = open_loop_input + P.flatten()

        self.cmd(control_input)

    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim

        Parameters
        ----------
        msg : numpy.ndarray
        """
        self.pub.publish(BicycleCommandMsg(*msg))

    def subscribe(self, msg):
        """
        callback fn for state listener.  Don't call me...

        Parameters
        ----------
        msg : :obj:`BicycleStateMsg`
        """
        for neighbor in msg.states:
            if neighbor.id == self.tb_id:
                self.state = np.array([msg.x, msg.y, msg.theta, msg.phi])
            else:
                self.add_neighbor(neighbor)

    def add_neighbor(self, neighbor):
        # TODO: have to add an update for if the neighbor no longer in range
        if neighbor.id in self.neighbors:
            self.neighbors[neighbor.id] = np.array(
                [neighbor.x, neighbor.y, neighbor.theta, neighbor.phi]
            )
        else:
            self.neighbors[neighbor.id] = np.array(
                [neighbor.x, neighbor.y, neighbor.theta, neighbor.phi]
            )

        for id, state in msg.neighbors.itmes():
            if id == self.id:
                continue
            if id in self.neighbors:
                self.neighbors.append(
                    np.array([neighbor.x, neighbor.y, neighbor.theta, neighbor.phi])
                )

    def shutdown(self):
        """
        Shutdown the controller node.
        """
        rospy.loginfo("Shutting Down")
        self.cmd((0, 0))
