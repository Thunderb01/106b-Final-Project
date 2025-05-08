################################################################################
#
# OccupancyGrid2d class listens for LaserScans and builds an occupancy grid.
#
################################################################################

import rospy
import tf2_ros
import tf

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import numpy as np

class OccupancyGrid2d(object):
    def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/grid_map_2d"

        # Load parameters.
        if not self.load_parameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.register_callbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up the map.
        self._map = np.zeros((self._x_num, self._y_num))

        self._initialized = True
        return True

    def load_parameters(self):
        # Random downsampling fraction, i.e. only keep this fraction of rays.
        if not rospy.has_param("~random_downsample"):
            return False
        self._random_downsample = rospy.get_param("~random_downsample")

        # Dimensions and bounds.
        # TODO! You'll need to set values for class variables called:
        # -- self._x_num
        if not rospy.has_param("~x/num"):
            return False
        self._x_num = rospy.get_param("~x/num")

        # -- self._x_min
        if not rospy.has_param("~x/min"):
            return False
        self._x_min = rospy.get_param("~x/min")

        # -- self._x_max
        if not rospy.has_param("~x/max"):
            return False
        self._x_max = rospy.get_param("~x/max")

        # -- self._x_res # The resolution in x. Note: This isn't a ROS parameter. What will you do instead?
        self._x_res = (self._x_max - self._x_min) / self._x_num

        # -- self._y_num
        if not rospy.has_param("~y/num"):
            return False
        self._y_num = rospy.get_param("~y/num")

        # -- self._y_min
        if not rospy.has_param("~y/min"):
            return False
        self._y_min = rospy.get_param("~y/min")

        # -- self._y_max
        if not rospy.has_param("~y/max"):
            return False
        self._y_max = rospy.get_param("~y/max")

        # -- self._y_res # The resolution in y. Note: This isn't a ROS parameter. What will you do instead?
        self._y_res = (self._y_max - self._y_min) / self._y_num

        # Update parameters.
        if not rospy.has_param("~update/occupied"):
            return False
        self._occupied_update = self.ProbabilityToLogOdds(
            rospy.get_param("~update/occupied")
        )

        if not rospy.has_param("~update/occupied_threshold"):
            return False
        self._occupied_threshold = self.ProbabilityToLogOdds(
            rospy.get_param("~update/occupied_threshold")
        )

        if not rospy.has_param("~update/free"):
            return False
        self._free_update = self.ProbabilityToLogOdds(
            rospy.get_param("~update/free")
        )

        if not rospy.has_param("~update/free_threshold"):
            return False
        self._free_threshold = self.ProbabilityToLogOdds(
            rospy.get_param("~update/free_threshold")
        )

        # Topics.
        # TODO! You'll need to set values for class variables called:
        # -- self._sensor_topic
        if not rospy.has_param("~topics/sensor"):
            return False
        self._sensor_topic = rospy.get_param("~topics/sensor")

        # -- self._vis_topic
        if not rospy.has_param("~topics/vis"):
            return False
        self._vis_topic = rospy.get_param("~topics/vis")

        # Frames.
        # TODO! You'll need to set values for class variables called:
        # -- self._sensor_frame
        if not rospy.has_param("~frames/sensor"):
            return False
        self._sensor_frame = rospy.get_param("~frames/sensor")
        # -- self._fixed_frame
        if not rospy.has_param("~frames/fixed"):
            return False
        self._fixed_frame = rospy.get_param("~frames/fixed")

        return True

    def register_callbacks(self):
        # Subscriber.
        self._sensor_sub = rospy.Subscriber(self._sensor_topic,
                                            LaserScan,
                                            self.sensor_callback,
                                            queue_size=1)

        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        Marker,
                                        queue_size=10)

        return True
    
    def get_limits(self):
        """
        Gets the limits of the map.

        Returns:
            A tuple containing the x and y limits of the map.
            In the format (x_min, x_max, y_min, y_max).
        """
        return (self._x_min, self._x_max, self._y_min, self._y_max)

    # Callback to process sensor measurements.
    def sensor_callback(self, msg):
        if not self._initialized:
            rospy.logerr("%s: Was not initialized.", self._name)
            return

        # Get our current pose from TF.
        try:
            pose = self._tf_buffer.lookup_transform(
                self._fixed_frame, self._sensor_frame, rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Writes an error message to the ROS log but does not raise an exception
            rospy.logerr("%s: Could not extract pose from TF.", self._name)
            return

        # Extract x, y coordinates and heading (yaw) angle of the turtlebot, 
        # assuming that the turtlebot is on the ground plane.
        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y
        if abs(pose.transform.translation.z) > 0.05:
            rospy.logwarn("%s: Turtlebot is not on ground plane.", self._name)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])
        if abs(roll) > 0.1 or abs(pitch) > 0.1:
            rospy.logwarn("%s: Turtlebot roll/pitch is too large.", self._name)

        # Loop over all ranges in the LaserScan.
        for idx, r in enumerate(msg.ranges):
            # Randomly throw out some rays to speed this up.
            if np.random.rand() > self._random_downsample:
                continue
            elif np.isnan(r):
                continue

            # Get angle of this ray in fixed frame.
            # TODO!
            angle_fixed = msg.angle_min + idx * msg.angle_increment + yaw

            # Throw out this point if it is too close or too far away.
            if r > msg.range_max:
                rospy.logwarn("%s: Range %f > %f was too large.",
                              self._name, r, msg.range_max)
                continue
            if r < msg.range_min:
                rospy.logwarn("%s: Range %f < %f was too small.",
                              self._name, r, msg.range_min)
                continue

            # Walk along this ray from the scan point to the sensor.
            # Update log-odds at each voxel along the way.
            # Only update each voxel once.
            # The occupancy grid is stored in self._map
            # TODO!
            x_final = sensor_x + r * np.cos(angle_fixed)
            y_final = sensor_y + r * np.sin(angle_fixed)
   
            # All of these voxels are measured free (only last one is measured occupied)
           
            step_size = -np.sqrt(self._x_res**2 + self._y_res**2)
            voxels = []
            for r_block in np.arange(r, 0, step_size):
                x = sensor_x + r_block * np.cos(angle_fixed)
                y = sensor_y + r_block * np.sin(angle_fixed)

                voxel = self.point_to_voxel(x, y) 
                if voxel not in voxels:
                    voxels.append(voxel)
          
                
            voxel_final = self.point_to_voxel(x_final, y_final)
            self._map[voxel_final] = min(self._map[voxel_final] + self._occupied_update, self._occupied_threshold)
            for voxel in voxels:
                if voxel != voxel_final:
                    self._map[voxel] = max(self._map[voxel] + self._free_update, self._free_threshold)
                

        # Visualize.
        self.visualize()

    # Convert (x, y) coordinates in fixed frame to grid coordinates.
    def point_to_voxel(self, x, y):
        grid_x = int((x - self._x_min) / self._x_res)
        grid_y = int((y - self._y_min) / self._y_res)

        return (grid_x, grid_y)

    # Get the center point (x, y) corresponding to the given voxel.
    def get_voxel_center(self, ii, jj):
        center_x = self._x_min + (0.5 + ii) * self._x_res
        center_y = self._y_min + (0.5 + jj) * self._y_res

        return (center_x, center_y)
    
    def get_voxel_log_odds(self, voxel):
        """
        Safely returns the log-odds value of a voxel in the occupancy grid.

        Args:
            voxel (tuple): (grid_x, grid_y) index

        Returns:
            float: log-odds value at that voxel, or None if out of bounds
        """
        ii, jj = voxel
        if 0 <= ii < self._x_num and 0 <= jj < self._y_num:
            return self._map[ii, jj]
        else:
            return None
        
    def get_voxel_neighbors(self, voxel, connectivity=4):
        """
        Returns neighboring voxels of a given voxel.

        Args:
            voxel (tuple): (grid_x, grid_y) index
            connectivity (int): 4 or 8 (default: 8)

        Returns:
            List of neighbor voxel indices (tuples)
        """
        ii, jj = voxel
        neighbors = []

        # Define neighbor offsets (4- or 8-connected)
        if connectivity == 4:
            offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        elif connectivity == 8:
            offsets = [(-1, -1), (-1, 0), (-1, 1),
                    (0, -1),          (0, 1),
                    (1, -1),  (1, 0), (1, 1)]
        else:
            raise ValueError("Connectivity must be 4 or 8")

        for dx, dy in offsets:
            ni, nj = ii + dx, jj + dy
            if 0 <= ni < self._x_num and 0 <= nj < self._y_num:
                neighbors.append((ni, nj))

        return neighbors
    
    def get_surrounding_obstacles(self, voxel, radius=1, is_point=False):
        """
        Returns surrounding obstacles within a given radius - in the shape of a diamond.

        Args:
            voxel (tuple): (grid_x, grid_y) index
            radius (int): radius to search for obstacles

        Returns:
            List of obstacle voxel indices and distances (tuples: (voxel, distance))
        """
        ii, jj = voxel
        obstacles = []

        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) + abs(dy) <= radius:
                    ni, nj = ii + dx, jj + dy
                    if 0 <= ni < self._x_num and 0 <= nj < self._y_num:
                        if self.is_voxel_occupied((ni, nj)):
                            dist = np.sqrt(dx**2 + dy**2)
                            obstacles.append(((ni, nj), dist))
        if not obstacles:
            return []
        # Sort by distance
        obstacles.sort(key=lambda x: x[1])

        # Return only the voxel indices, not the distances
        return obstacles if not is_point else [(self.get_voxel_center(vox), dist) for vox, dist in obstacles]
    
    def is_voxel_free(self, voxel):
        """
        Check if a voxel is free.

        Args:
            voxel (tuple): (grid_x, grid_y) index

        Returns:
            bool: True if the voxel is free, False otherwise
        """
        return self._map[voxel] < self._free_threshold
    
    def is_voxel_occupied(self, voxel):
        """
        Check if a voxel is occupied.
        Args:
            voxel (tuple): (grid_x, grid_y) index
        Returns:
            bool: True if the voxel is occupied, False otherwise
        """
        return self._map[voxel] > self._occupied_threshold
    
    def is_voxel_unknown(self, voxel):
        """
        Check if a voxel is unknown.
        Args:
            voxel (tuple): (grid_x, grid_y) index
        Returns:
            bool: True if the voxel is unknown, False otherwise
        """
        return self._map[voxel] >= self._free_threshold and self._map[voxel] <= self._occupied_threshold
    
    @staticmethod
    def merge_maps(map1, map2, clip_min=-10.0, clip_max=10.0):
        """
        Merges two log-odds maps via element-wise addition.
        Args:
            map1 (numpy.ndarray): First log-odds map.
            map2 (numpy.ndarray): Second log-odds map.
            clip_min (float): Minimum value to clip the result.
            clip_max (float): Maximum value to clip the result.
        Returns:
            numpy.ndarray: Merged log-odds map.
        Raises: 
            ValueError if the shapes of the maps do not match.
        """
        if map1.shape != map2.shape:
            raise ValueError("Map sizes do not match")
        return np.clip(map1 + map2, clip_min, clip_max) # log odds are added together beacuse log rules, we are multiplying the probabilities



    # Convert between probabity and log-odds.
    def ProbabilityToLogOdds(self, p):
        return np.log(p / (1.0 - p))

    def LogOddsToProbability(self, l):
        return 1.0 / (1.0 + np.exp(-l))

    # Colormap to take log odds at a voxel to a RGBA color.
    def colormap(self, ii, jj):
        p = self.LogOddsToProbability(self._map[ii, jj])

        c = ColorRGBA()
        c.r = p
        c.g = 0.1
        c.b = 1.0 - p
        c.a = 0.75
        return c

    # Visualize the map as a collection of flat cubes instead of
    # as a built-in OccupancyGrid message, since that gives us more
    # flexibility for things like color maps and stuff.
    # See http://wiki.ros.org/rviz/DisplayTypes/Marker for a brief tutorial.
    def visualize(self):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self._fixed_frame
        m.ns = "map"
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = self._x_res
        m.scale.y = self._y_res
        m.scale.z = 0.01

        for ii in range(self._x_num):
            for jj in range(self._y_num):
                p = Point(0.0, 0.0, 0.0)
                (p.x, p.y) = self.get_voxel_center(ii, jj)

                m.points.append(p)
                m.colors.append(self.colormap(ii, jj))

        self._vis_pub.publish(m)
