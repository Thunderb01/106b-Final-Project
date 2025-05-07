#! /usr/bin/env python
"""
Code for comparing the maps of the robots in the swarm.
"""
import rospy
import numpy as np
from swarm_explorer.msg import ExplorerState


class AbstractMap:
    def __init__(self, latest_map):
        """
        Initializes the map with the given data.
        """
        self.latest_map = latest_map

    def get_cell(self, idx):
        pass

    def set_cell(self, idx, value):
        pass

    def all_cells_known(self):
        """
        Returns True if all cells in the map are known.
        """
        pass


class OccupancyMap(AbstractMap):
    def __init__(self, latest_map):
        """
        Initializes the occupancy map with the given data.
        """
        super().__init__(latest_map)
        self.latest_map = np.array(latest_map.data).reshape(
            latest_map.info.height, latest_map.info.width
        )

    def get_cell(self, idx):
        return self.latest_map[idx]

    def set_cell(self, idx, value):
        self.latest_map[idx] = value

    def all_cells_known(self):
        return np.all(self.latest_map != -1)


# TODO: update this map to work for SLAM data and whatever we need for this
class SLAMMap(AbstractMap):
    def __init__(self, latest_map):
        """
        Initializes the SLAM map with the given data.
        """
        super().__init__(latest_map)
        self.latest_map = np.array(latest_map.data).reshape(
            latest_map.info.height, latest_map.info.width
        )

    def get_cell(self, idx):
        return self.latest_map[idx]

    def set_cell(self, idx, value):
        self.latest_map[idx] = value


class FrontierUpdater:
    def __init__(self, robot_id, map_type, latest_map):
        """
        Initializes the FrontierUpdater with info from the parameter server.
        """
        self.robot_id = robot_id
        self.map_type = map_type
        if map_type == "occupancy":
            self.amap = OccupancyMap(latest_map)
        elif map_type == "slam":
            self.amap = SLAMMap(latest_map)
        else:
            rospy.logerr("Invalid map type specified. Exiting.")
            rospy.signal_shutdown("Invalid map type specified.")
            return
        # TODO: fill in the rest of this class
        pass

    def update_frontiers(self, neighbor_map: AbstractMap):
        """
        Updates the frontiers of the robots in the swarm.
        """
        # Get the map from the message
        

        # Update the frontiers of the robots in the swarm
