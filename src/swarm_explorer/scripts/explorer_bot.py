#!/usr/bin/env python
import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_conversions


def main():
    rospy.init_node("explorer_bot")
    bot_id = rospy.get_param("~robot_id")  # 1-based

    poses = rospy.get_param("/initial_poses")  # list of dicts
    my_pose = poses[bot_id - 1]
    x, y, theta = my_pose["x"], my_pose["y"], my_pose["theta"]

    # publish static map→odom
    br = StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = f"robot_{bot_id}/odom"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0
    quat = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    br.sendTransform(t)

    # …rest of your setup (subscribers, PD loop, etc.)…

    rospy.spin()


if __name__ == "__main__":
    main()
