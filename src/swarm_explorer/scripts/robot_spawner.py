#!/usr/bin/env python3
import rospy
import subprocess
import time
import os
from geometry_msgs.msg import TransformStamped
import tf2_ros

class RobotSpawner:
    def __init__(self):
        rospy.init_node('robot_spawner')
        self.robot_positions = rospy.get_param('~robot_positions')
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
    def spawn_robots(self):
        for robot in self.robot_positions:
            robot_id = robot['id']
            x = robot['x']
            y = robot['y']
            z = robot['z']
            yaw = robot['yaw']
            
            # Create robot namespace
            namespace = f"robot_{robot_id}"
            
            # Load robot description
            robot_description_cmd = f"$(find xacro)/xacro '$(find turtlebot3_description)/urdf/turtlebot3_burger.urdf.xacro'"
            rospy.set_param(f"/{namespace}/robot_description", 
                          subprocess.check_output(robot_description_cmd, shell=True).decode())
            
            # Spawn robot in Gazebo
            spawn_cmd = f"rosrun gazebo_ros spawn_model -param /{namespace}/robot_description -urdf -model {namespace} -x {x} -y {y} -z {z} -Y {yaw} -robot_namespace /{namespace}"
            subprocess.Popen(spawn_cmd, shell=True)
            
            # Set up TF transform
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "map"
            transform.child_frame_id = f"{namespace}/odom"
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            
            # Convert yaw to quaternion
            import tf_conversions
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(transform)
            
            # Set up robot controller parameters
            rospy.set_param(f"/{namespace}/diff_drive_controller/odom_frame_id", f"{namespace}/odom")
            rospy.set_param(f"/{namespace}/diff_drive_controller/base_frame_id", f"{namespace}/base_footprint")
            rospy.set_param(f"/{namespace}/diff_drive_controller/publish_rate", 50.0)
            
            # Launch robot state publisher
            state_pub_cmd = f"rosrun robot_state_publisher robot_state_publisher __name:=robot_state_publisher_{robot_id} __ns:={namespace}"
            subprocess.Popen(state_pub_cmd, shell=True)
            
            # Launch joint state publisher
            joint_pub_cmd = f"rosrun joint_state_publisher joint_state_publisher __name:=joint_state_publisher_{robot_id} __ns:={namespace}"
            subprocess.Popen(joint_pub_cmd, shell=True)
            
            # Launch explorer bot
            explorer_cmd = f"rosrun swarm_explorer explorer_bot.py __name:=explorer_bot_{robot_id} __ns:={namespace} _robot_id:={robot_id}"
            subprocess.Popen(explorer_cmd, shell=True)
            
            # Launch mapping node
            mapping_cmd = f"rosrun mapping mapping_node.py __name:=mapping_node_{robot_id} __ns:={namespace}"
            subprocess.Popen(mapping_cmd, shell=True)
            
            # Wait a bit between spawns to avoid race conditions
            time.sleep(1.0)
            
    def run(self):
        try:
            self.spawn_robots()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    spawner = RobotSpawner()
    spawner.run() 