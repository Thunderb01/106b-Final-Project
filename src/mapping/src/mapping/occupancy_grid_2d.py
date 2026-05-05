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
from swarm_explorer.msg import ExplorerMapMsg
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

import numpy as np

class OccupancyGrid2d(object):
    def __init__(self):
        self._intialized = False
        self._update_count = 0
        self._last_vis_pub_time = None

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    @classmethod
    def from_msg(cls, msg: ExplorerMapMsg):
        """
        Initializes the OccupancyGrid2d object with a given ExplorerMapMsg message.

        Args:
            msg (ExplorerMapMsg): The occupancy grid message to initialize the object with.
        """
        obj = cls()
        obj.Initialize(setup_callbacks=False)
        obj._map = np.array(msg.grid.data).reshape((msg.grid.info.width, msg.grid.info.height))
        return obj

    # Initialization and loading parameters.
    def Initialize(self, setup_callbacks=True):
        self._name = rospy.get_name() + "/grid_map_2d"

        # Load parameters.
        if not self.load_parameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        
        if setup_callbacks and not self.register_callbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up the map.
        self._map = np.zeros((self._x_num, self._y_num))

        self._initialized = True
        return True

    def load_parameters(self):
        # List of (param_name, attr_name, transform) tuples:
        params = [
            # downsample
            ("random_downsample", "_random_downsample", lambda v: v),
            # x axis
            ("x/num",            "_x_num",            int),
            ("x/min",            "_x_min",            float),
            ("x/max",            "_x_max",            float),
            # y axis
            ("y/num",            "_y_num",            int),
            ("y/min",            "_y_min",            float),
            ("y/max",            "_y_max",            float),
            # update
            ("update/occupied",            "_occupied_update",          self.ProbabilityToLogOdds),
            ("update/occupied_threshold",  "_occupied_threshold",       self.ProbabilityToLogOdds),
            ("update/free",                "_free_update",              self.ProbabilityToLogOdds),
            ("update/free_threshold",      "_free_threshold",           self.ProbabilityToLogOdds),
            # topics
            ("topics/sensor",    "_sensor_topic",     lambda v: v),
            ("topics/vis",       "_vis_topic",        lambda v: v),
            # frames
            ("frames/sensor",    "_sensor_frame",     lambda v: v),
            ("frames/fixed",     "_fixed_frame",      lambda v: v),
        ]

        node_ns = rospy.get_name()            # e.g. "/robot_1/robot_1/explorer_bot"
        base = rospy.get_name()

        for param_name, attr, cast in params:
            key = f"{base}/{param_name}"
            if not rospy.has_param(key):
                rospy.logerr(f"{node_ns}: missing param `{key}`")
                return False
            raw = rospy.get_param(key)
            try:
                setattr(self, attr, cast(raw))
            except Exception as e:
                rospy.logerr(f"{node_ns}: failed to cast param `{key}` ({raw}) -> {e}")
                return False

        # now compute resolutions
        try:
            self._x_res = (self._x_max - self._x_min) / self._x_num
            self._y_res = (self._y_max - self._y_min) / self._y_num
        except Exception as e:
            rospy.logerr(f"{node_ns}: error computing resolutions -> {e}")
            return False

        # Optional visualization load controls.
        self._vis_publish_hz = float(rospy.get_param(f"{base}/vis/publish_hz", 2.0))
        if self._vis_publish_hz <= 0.0:
            self._vis_publish_hz = 2.0
        # Max-range scans usually indicate "no obstacle hit"; avoid marking
        # endpoints occupied when range is very close to range_max.
        self._max_range_hit_margin = float(
            rospy.get_param(f"{base}/scan/max_range_hit_margin", 0.05)
        )
        if self._max_range_hit_margin < 0.0:
            self._max_range_hit_margin = 0.0
        # Cells with near-zero log-odds are considered "unknown" for
        # frontier extraction and map sharing.
        self._unknown_log_odds_eps = float(
            rospy.get_param(f"{base}/unknown/log_odds_eps", 0.15)
        )
        if self._unknown_log_odds_eps < 0.0:
            self._unknown_log_odds_eps = 0.0

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
        # base_scan is above the ground plane in Turtlebot models, so only warn for
        # unexpectedly large vertical offsets.
        if abs(pose.transform.translation.z) > 0.30:
            rospy.logwarn_throttle(2.0, "%s: Turtlebot sensor z-offset is large (z=%.3f).",
                                   self._name, pose.transform.translation.z)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])
        if abs(roll) > 0.1 or abs(pitch) > 0.1:
            rospy.logwarn("%s: Turtlebot roll/pitch is too large.", self._name)

        # Ensure the robot cell itself is treated as free so frontier search
        # can grow from the current pose instead of stalling at unknown.
        sensor_voxel = self.point_to_voxel(sensor_x, sensor_y)
        self._map[sensor_voxel] = max(
            self._map[sensor_voxel] + self._free_update, self._free_threshold
        )
        
        # try:
        #     tframe = self._tf_buffer.lookup_transform("map", self.sensor_frame, rospy.Time(0), rospy.Duration(0.1))
        #     rospy.loginfo(f"[{rospy.get_name()}] TF translation: {tframe.transform.translation}")
        # except Exception as e:
        #     rospy.logwarn(f"TF lookup failed: {e}")

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
                # rospy.logwarn("%s: Range %f > %f was too large.",
                #               self._name, r, msg.range_max)
                continue
            if r < msg.range_min:
                # rospy.logwarn("%s: Range %f < %f was too small.",
                #               self._name, r, msg.range_min)
                continue

            # Walk along this ray from the scan point to the sensor.
            # Update log-odds at each voxel along the way.
            # Only update each voxel once.
            # The occupancy grid is stored in self._map
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
            # TODO: Alex will fix this. Right alex....? ong no🧢
            if voxel_final[0] >= self._x_num or voxel_final[1] >= self._y_num:
                # print("voxel_final out of bounds")
                continue
            else:
                hit_obstacle = (msg.range_max - r) > self._max_range_hit_margin
                if hit_obstacle:
                    # Endpoint is a measured obstacle hit.
                    self._map[voxel_final] = min(
                        self._map[voxel_final] + self._occupied_update,
                        self._occupied_threshold,
                    )
                for voxel in voxels:
                    if voxel[0] >= self._x_num or voxel[1] >= self._y_num:
                        rospy.logwarn("%s: Voxel in list out of bounds: (%d, %d) and voxel_final: (%d, %d)", 
                                    self._name, voxel[0], voxel[1], voxel_final[0], voxel_final[1])
                        continue
                    # For max-range/no-hit rays, treat endpoint as free too.
                    if hit_obstacle and voxel == voxel_final:
                        continue
                    else:
                        self._map[voxel] = max(self._map[voxel] + self._free_update, self._free_threshold)
                

        # Visualize.
        self.visualize()
        self._update_count += 1

    def get_update_count(self):
        return self._update_count

    def is_fully_known(self):
        """
        Check if the map is fully known.
        Returns:
            bool: True if there are no unknown cells in the map, False otherwise.
        """
        # A cell is unknown if its log-odds value is between free_threshold and occupied_threshold
        return False # TODO: we aint there yet 
        unknown_mask = (self._map >= self._free_threshold) & (self._map <= self._occupied_threshold)
        return not np.any(unknown_mask)
    
    def to_msg(self):
        """
        Converts the occupancy grid to a ROS ExplorerMapMsg message. (doesn't set id)
        Returns:
            ExplorerMapMsg: The occupancy grid message.
        """
        msg = ExplorerMapMsg()
        msg.grid = OccupancyGrid()
        msg.grid.header = Header()
        msg.grid.header.stamp = rospy.Time.now()
        msg.grid.header.frame_id = self._fixed_frame

        msg.grid.info.resolution = self._x_res
        msg.grid.info.width = self._x_num
        msg.grid.info.height = self._y_num

        msg.grid.info.origin = Pose()
        msg.grid.info.origin.position.x = self._x_min
        msg.grid.info.origin.position.y = self._y_min
        msg.grid.info.origin.position.z = 0.0
        msg.grid.info.origin.orientation.w = 1.0  # no rotation

        # Convert log-odds map to probability (0-100) or -1 for unknown.
        # For sharing/fusion, treat only near-zero log-odds as unknown so
        # partially observed evidence is transmitted to neighbors.
        data = []
        for ii in range(self._x_num):
            for jj in range(self._y_num):
                logodds = self._map[ii, jj]
                if abs(logodds) < self._unknown_log_odds_eps:
                    data.append(-1)
                else:
                    p = self.LogOddsToProbability(logodds)
                    data.append(int(100 * p))

        msg.grid.data = data
        return msg

    # Convert (x, y) coordinates in fixed frame to grid coordinates.
    def point_to_voxel(self, x, y):
        grid_x = int((x - self._x_min) / self._x_res)
        grid_y = int((y - self._y_min) / self._y_res)

        # Ensure indices are within bounds
        grid_x = max(0, min(grid_x, self._x_num - 1))
        grid_y = max(0, min(grid_y, self._y_num - 1))

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

    def get_surrounding_obstacles(self, position, radius=1, is_point=False):
        """
        Returns occupied voxels within a diamond (Manhattan radius) in **cell** units,
        whose circumscribing disk in **meters** is approximately ``radius``.

        Args:
            position (tuple(float)): (x, y) in world / map meters
            radius (float): search radius in **meters** (same units as ``position``)

        Returns:
            List of ``(voxel_index, dist_m)`` or ``(world_point, dist_m)`` when
            ``is_point`` is True. ``dist_m`` is the Euclidean distance in **meters**
            from ``position`` to the **center** of the obstacle voxel (or from robot
            cell center consistency with ``get_voxel_center``).
        """
        voxel = self.point_to_voxel(position[0], position[1])
        ii, jj = voxel
        obstacles = []
        # Diamond radius in whole cells so the search region covers ``radius`` meters
        # (conservative when x_res != y_res: use the finer resolution).
        cell = min(self._x_res, self._y_res)
        radius_cells = max(1, int(np.ceil(radius / cell)))

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if abs(dx) + abs(dy) <= radius_cells:
                    ni, nj = ii + dx, jj + dy
                    if 0 <= ni < self._x_num and 0 <= nj < self._y_num:
                        if self.is_voxel_occupied((ni, nj)):
                            cx, cy = self.get_voxel_center(ni, nj)
                            dist_m = float(
                                np.hypot(position[0] - cx, position[1] - cy)
                            )
                            if dist_m <= radius:
                                obstacles.append(((ni, nj), dist_m))
        if not obstacles:
            return []
        obstacles.sort(key=lambda x: x[1])

        return obstacles if not is_point else [(self.get_voxel_center(*vox), dist) for vox, dist in obstacles]
    
    def is_voxel_free(self, voxel):
        """
        Check if a voxel is free.

        Args:
            voxel (tuple): (grid_x, grid_y) index

        Returns:
            bool: True if the voxel is free, False otherwise
        """
        return self._map[voxel[0], voxel[1]] < self._free_threshold
    
    def is_voxel_occupied(self, voxel):
        """
        Check if a voxel is occupied.
        Args:
            voxel (tuple): (grid_x, grid_y) index
        Returns:
            bool: True if the voxel is occupied, False otherwise
        """
        return self._map[voxel[0], voxel[1]] > self._occupied_threshold
    
    def is_voxel_unknown(self, voxel):
        """
        Check if a voxel is unknown.
        Args:
            voxel (tuple): (grid_x, grid_y) index
        Returns:
            bool: True if the voxel is unknown, False otherwise
        """
        return abs(self._map[voxel[0], voxel[1]]) < self._unknown_log_odds_eps
    
    @staticmethod
    def merge_maps(map1, map2, clip_min=-10.0, clip_max=10.0):
        """
        Merges two log-odds maps via element-wise addition.
        Args:
            map1 (OccupancyGrid2d): First log-odds map.
            map2 (OccupancyGrid2d): Second log-odds map.
            clip_min (float): Minimum value to clip the result.
            clip_max (float): Maximum value to clip the result.
        Returns:# Merge current map with neighbor's map if needed
        merged_map = self.merge_maps(self.amap, neighbor_map)
            numpy.ndarray: Merged log-odds map.
        Raises: 
            ValueError if the shapes of the maps do not match.
        """
        if map1._map.shape != map2._map.shape:
            raise ValueError("Map sizes do not match")
        merged_data = np.clip(map1._map + map2._map, clip_min, clip_max) # log odds are added together beacuse log rules, we are multiplying the probabilities
        return merged_data

    # Convert between probabity and log-odds.
    def ProbabilityToLogOdds(self, p):
        return np.log(p / (1.0 - p))

    def LogOddsToProbability(self, l):
        return 1.0 / (1.0 + np.exp(-l))

    # Colormap to take log odds at a voxel to a RGBA color.
    def colormap(self, ii, jj):
        log_odds = self._map[ii, jj]
        # For visualization, treat only near-zero log-odds as unknown.
        # This avoids rendering nearly all updated cells as unknown gray.
        if abs(log_odds) < 1e-3:
            c = ColorRGBA()
            c.r = 0.35
            c.g = 0.35
            c.b = 0.35
            c.a = 0.12
            return c

        p = self.LogOddsToProbability(log_odds)

        c = ColorRGBA()
        c.r = p
        c.g = 0.1
        c.b = 1.0 - p
        c.a = 0.80
        return c

    # Visualize the map as a collection of flat cubes instead of
    # as a built-in OccupancyGrid message, since that gives us more
    # flexibility for things like color maps and stuff.
    # See http://wiki.ros.org/rviz/DisplayTypes/Marker for a brief tutorial.
    def visualize(self):
        # Throttle marker publishing to keep RViz responsive on slower machines.
        now = rospy.Time.now()
        if self._last_vis_pub_time is not None:
            dt = (now - self._last_vis_pub_time).to_sec()
            if dt < (1.0 / self._vis_publish_hz):
                return
        self._last_vis_pub_time = now

        m = Marker()
        m.header.stamp = now
        m.header.frame_id = self._fixed_frame
        m.ns = "map"
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.a = 1.0
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
