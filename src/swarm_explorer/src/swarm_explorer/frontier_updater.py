#! /usr/bin/env python
"""
Code for comparing the maps of the robots in the swarm.
"""
import rospy
import numpy as np
from swarm_explorer.msg import ExplorerStateMsg
from mapping.occupancy_grid_2d import OccupancyGrid2d

class Frontier:
    def __init__(self):
        self.cells = []  # List of cells in the frontier
        self.size = 0  # Size of the frontier
    
    def get_centroid(self):
        """
        Calculate the centroid of the frontier.
        """
        if not self.cells:
            return None
        x_sum = sum(cell[0] for cell in self.cells)
        y_sum = sum(cell[1] for cell in self.cells)
        self.centroid = (x_sum / len(self.cells), y_sum / len(self.cells))
        return self.centroid
    
    def add_cell(self, cell):
        """
        Add a cell to the frontier.
        """
        self.cells.append(cell)
        self.size += 1

    def distance_to_frontier(self, cell):
        """
        Calculate the distance from a cell to the nearest frontier.

        Args:
            cell: The cell to check
            frontier: The frontier region

        Returns:
            Distance to the nearest frontier
        """
        distances = [np.linalg.norm(np.array(cell) - np.array(f)) for f in self.cells]
        return min(distances) if distances else float("inf")

class FrontierUpdater:
    def __init__(self, robot_id, occupancy_map, frontier_dist_wt, frontier_size_wt):
        """
        Initializes the FrontierUpdater with info from the parameter server.
        """
        self.robot_id: int = robot_id
        self.occupancy_map: OccupancyGrid2d = occupancy_map
        # Weights for frontier selection
        self.frontier_dist_wt: float = frontier_dist_wt
        self.frontier_size_wt: float = frontier_size_wt

        # obsolete
        # if map_type == "occupancy":
        #     self.amap = OccupancyMap(latest_map)
        # elif map_type == "slam":
        #     self.amap = SLAMMap(latest_map)
        # else:
        #     rospy.logerr("Invalid map type specified. Exiting.")
        #     rospy.signal_shutdown("Invalid map type specified.")
        #     return
        self.frontiers = (
            []
        )  # List to store detected frontiers (initialize list of frontiers)
        self.visited = set()  # Set to track visited cells during frontier search

    def update_frontiers(self, current_position):
        """
        Updates the frontiers of the robots in the swarm using neighbor's map data.

        Args:
            current_position: The current position of the robot (x, y) in meters
        """

        # Convert position to voxel coordinates if needed
        current_cell = self.occupancy_map.point_to_voxel(
            current_position[0], current_position[1]
        )
        if current_cell is None:
            rospy.logerr("Current position is out of bounds.")
            return

        # Perform frontier search
        self.frontiers = self.find_frontiers(current_cell, self.occupancy_map)

        # Filter and process frontiers (optional)
        self.filter_frontiers()

        # Publish frontiers if needed
        self.publish_frontiers()

    def find_frontiers(self, start_cell, map_data: OccupancyGrid2d):
        """
        Implementation of Algorithm 1 - Frontier cell search.

        Args:
            start_cell: Starting cell (current robot position)
            map_data: Map data to search through

        Returns:
            List of frontier regions
        """
        frontiers = []
        queue = [start_cell]
        self.visited = set()

        while queue:
            cell = queue.pop(0)  # dequeue cell

            if cell in self.visited:
                continue

            self.visited.add(cell)

            # Get 4-connected neighbors (up, down, left, right)
            neighbors = map_data.get_voxel_neighbors(cell, connectivity=4)

            for neighbor in neighbors:
                # Skip if already visited
                if neighbor in self.visited:
                    continue
                

                if map_data.is_voxel_free(neighbor):
                    queue.append(neighbor)
                elif map_data.is_voxel_unknown(neighbor):
                    # Found a frontier cell, now find the entire frontier region
                    frontier_region = self.find_frontier_region(neighbor, map_data)
                    if frontier_region:
                        frontiers.append(frontier_region)

        return frontiers

    def find_frontier_region(self, start_cell, map_data: OccupancyGrid2d):
        """
        Implementation of Algorithm 2 - Frontier region connectivity search

        Finds a complete frontier region starting from a frontier cell.
        Similar to the region growing algorithm mentioned in your pseudocode.
        """
        region = Frontier()
        region.add_cell(start_cell)  # Add the starting cell to the region
        queue = [start_cell]
        visited = set()

        while queue:
            cell = queue.pop(0)

            if cell in visited:
                continue

            visited.add(cell)
            region.add_cell(cell)

            neighbors = map_data.get_voxel_neighbors(cell, connectivity=4)

            for neighbor in neighbors:
                if map_data.is_voxel_unknown(neighbor) and neighbor not in visited:
                    queue.append(neighbor)

        return region  # if len(region) > MIN_FRONTIER_SIZE else None

    

    def get_best_frontier(self, point):
        """
        Get the best frontier to a given cell.
        Args:
            cell: The cell to check
        Returns:
            Closest frontier region
        """
        cell = self.occupancy_map.point_to_voxel(point[0], point[1])
        closest_frontier = None
        min_cost = float("inf")

        for frontier in self.frontiers:
            distance = frontier.distance_to_frontier(cell)
            size = frontier.size
            cost = self.frontier_dist_wt * distance - self.frontier_size_wt * size
            if cost < min_cost:
                min_cost = cost
                closest_frontier = frontier

        return closest_frontier

    def get_closest_frontier_robot(self, robot_id):
        """
        Get the closest frontier to a given robot.

        Args:
            robot_id: The ID of the robot

        Returns:
            Closest frontier region
        """
        # Assuming you have a way to get the robot's position
        robot_position = self.get_robot_position(robot_id)
        cell = self.world_to_grid(robot_position)

        return self.get_closest_frontier(cell)

    # Helper methods

    def filter_frontiers(self):
        """Filter out small or unreachable frontiers."""
        # Add your filtering logic here
        pass

    def publish_frontiers(self):
        """Publish the detected frontiers to other robots/nodes."""
        # Implementation depends on your messaging system
        pass
