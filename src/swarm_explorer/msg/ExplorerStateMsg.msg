# State of an explorer bot in the swarm at a timestamp
uint8               robot_id
# Actual measured state
nav_msgs/Odometry   odometry
# Robot pose in the shared map frame (filled by sender for comm range / flocking)
geometry_msgs/PoseStamped map_pose
# Calculated states (legacy; prefer map_pose for world-frame position)
geometry_msgs/Pose  pose
geometry_msgs/Twist flock_twist
geometry_msgs/Twist frontier_twist
