# State of an explorer bot in the swarm at a timestamp
std_msgs/Header header
uint8           robot_id
BicycleStateMsg state
MapMsg          map # TODO: create map msg, also allow for SLAM data at some point
