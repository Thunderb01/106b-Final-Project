#! /usr/bin/env python
"""
Code for comparing the maps of the robots in the swarm.
"""
import rospy
import numpy as np
from swarm_explorer.msg import ExplorerState
from mapping.src.occupancy_grid_2d import OccupancyGrid2d




class FrontierUpdater:
    def __init__(self, robot_id, occupancy_map):
        """
        Initializes the FrontierUpdater with info from the parameter server.
        """
        self.robot_id = robot_id
        self.occupancy_map: OccupancyGrid2d = occupancy_map

        # obsolete
        # if map_type == "occupancy":
        #     self.amap = OccupancyMap(latest_map)
        # elif map_type == "slam":
        #     self.amap = SLAMMap(latest_map)
        # else:
        #     rospy.logerr("Invalid map type specified. Exiting.")
        #     rospy.signal_shutdown("Invalid map type specified.")
        #     return
        self.frontiers = [] # List to store detected frontiers (initialize list of frontiers)
        self.visited = set() # Set to track visited cells during frontier search

    def update_frontiers(self, neighbor_map: OccupancyGrid2d):
        """
        Updates the frontiers of the robots in the swarm using neighbor's map data.
        
        Args:
            neighbor_map: Map data from neighboring robots
        """
        # Merge current map with neighbor's map if needed
        merged_map = self.merge_maps(self.amap, neighbor_map)
        
        # Get current robot position (implementation depends on your setup)
        current_position = self.get_robot_position()  
        
        # Convert position to grid coordinates if needed
        current_cell = self.world_to_grid(current_position)
        
        # Perform frontier search
        self.frontiers = self.find_frontiers(current_cell, merged_map)
        
        # Filter and process frontiers (optional)
        self.filter_frontiers()
        
        # Publish frontiers if needed
        self.publish_frontiers()

    def find_frontiers(self, start_cell, map_data):
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
            cell = queue.pop(0) #dequeue cell
            
            if cell in self.visited:
                continue
                
            self.visited.add(cell)
            
            # Get 4-connected neighbors (up, down, left, right)
            neighbors = self.get_neighbors(cell, map_data)
            
            for neighbor in neighbors:
                # Skip if already visited
                if neighbor in self.visited:
                    continue
                    
                cell_value = map_data.get_cell_value(neighbor)
                
                if self.is_free(cell_value):
                    queue.append(neighbor)
                elif self.is_frontier(cell, cell_value):
                    # Found a frontier cell, now find the entire frontier region
                    frontier_region = self.find_frontier_region(neighbor, map_data)
                    if frontier_region:
                        frontiers.append(frontier_region)
        
        return frontiers

    def find_frontier_region(self, start_cell, map_data):
        """
        Implementation of Algorithm 2 - Frontier region connectivity search 

        Finds a complete frontier region starting from a frontier cell.
        Similar to the region growing algorithm mentioned in your pseudocode.
        """
        region = []
        queue = [start_cell]
        visited = set()
        
        while queue:
            cell = queue.pop(0)
            
            if cell in visited:
                continue
                
            visited.add(cell)
            region.append(cell)
            
            neighbors = self._get_neighbors(cell, map_data)
            
            for neighbor in neighbors:
                cell_value = map_data.get_cell_value(neighbor)
                if self._is_frontier_candidate(cell_value) and neighbor not in visited:
                    queue.append(neighbor)
        
        return region # if len(region) > MIN_FRONTIER_SIZE else None
    
    def distance_to_frontier(self, cell, frontier):
        """
        Calculate the distance from a cell to the nearest frontier.
        
        Args:
            cell: The cell to check
            frontier: The frontier region
            
        Returns:
            Distance to the nearest frontier
        """
        distances = [np.linalg.norm(np.array(cell) - np.array(f)) for f in frontier]
        return min(distances) if distances else float('inf')
    
    def get_closest_frontier(self, point):
        """
        Get the closest frontier to a given cell.
        
        Args:
            cell: The cell to check
            
        Returns:
            Closest frontier region
        """
        cell = self.occupancy_map.point_to_voxel(point)
        closest_frontier = None
        min_distance = float('inf')
        
        for frontier in self.frontiers:
            distance = self.distance_to_frontier(cell, frontier)
            if distance < min_distance:
                min_distance = distance
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

    def get_frontier_size(self, frontier):
        """
        Get the size of a frontier region.
        
        Args:
            frontier: The frontier region
            
        Returns:
            Size of the frontier
        """
        return len(frontier)

    def get_neighbors(self, cell, map_data):
        return map_data.get_neighbors(cell, connectivity=4)

    def is_free(self, logodds):
        return logodds is not None and logodds <= self.occupancy_map._free_threshold

    def is_frontier(self, cell, cell_value):
        return self.occupancy_map.is_unknown(cell_value) and any(
            self.is_free(self.occupancy_map.get_cell_value(n))
            for n in self.get_neighbors(cell, self.occupancy_map)
        )

    def filter_frontiers(self):
        """Filter out small or unreachable frontiers."""
        # Add your filtering logic here
        pass

    def publish_frontiers(self):
        """Publish the detected frontiers to other robots/nodes."""
        # Implementation depends on your messaging system
        pass
