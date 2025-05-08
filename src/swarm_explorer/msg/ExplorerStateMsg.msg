# State of an explorer bot in the swarm at a timestamp
uint8               robot_id
# Actual measured state
nav_msgs/Odometry   odometry
# Calculated states
geometry_msgs/Pose  pose
geometry_msgs/Twist flock_twist
geometry_msgs/Twist frontier_twist
