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
        Distance from voxel index ``cell`` to the nearest frontier cell, in **grid
        index units** (Euclidean norm of integer index differences).
        """
        distances = [np.linalg.norm(np.array(cell) - np.array(f)) for f in self.cells]
        return min(distances) if distances else float("inf")

    def min_world_distance_to(self, wx, wy, occ_map):
        """
        Minimum Euclidean distance in **meters** from world point (wx, wy) to the
        center of any frontier voxel (uses ``occ_map.get_voxel_center``).
        """
        best = float("inf")
        for f in self.cells:
            ii = int(round(f[0]))
            jj = int(round(f[1]))
            cx, cy = occ_map.get_voxel_center(ii, jj)
            best = min(best, float(np.hypot(wx - cx, wy - cy)))
        return best

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
        # Keep frontier choice distance-first; large regions only get a bounded
        # bonus so one giant unknown area does not attract every robot.
        self.frontier_min_region_size = int(
            rospy.get_param(
                "~frontier_min_region_size",
                rospy.get_param("/frontier_min_region_size", 5),
            )
        )
        self.frontier_size_bonus_scale_m = float(
            rospy.get_param(
                "~frontier_size_bonus_scale_m",
                rospy.get_param("/frontier_size_bonus_scale_m", 0.35),
            )
        )

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

    def _is_likely_free(self, cell, map_data: OccupancyGrid2d):
        """Treat negative log-odds as traversable free space."""
        log_odds = map_data.get_voxel_log_odds(cell)
        return log_odds is not None and log_odds < 0.0

    def _is_frontier_cell(self, cell, map_data: OccupancyGrid2d):
        if not map_data.is_voxel_unknown(cell):
            return False
        neighbors = map_data.get_voxel_neighbors(cell, connectivity=4)
        return any(self._is_likely_free(n, map_data) for n in neighbors)

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
        if len(self.frontiers) == 0:
            local_map = self.occupancy_map._map
            free_cells = int(np.sum(local_map < 0.0))
            unknown_cells = int(
                np.sum(
                    (local_map >= self.occupancy_map._free_threshold)
                    & (local_map <= self.occupancy_map._occupied_threshold)
                )
            )
            start_log_odds = self.occupancy_map.get_voxel_log_odds(current_cell)
            update_count = self.occupancy_map.get_update_count()
            rospy.logwarn_throttle(
                2.0,
                "Robot %d local frontier=0 (updates=%d, free=%d, unknown=%d, start_log_odds=%.3f)",
                self.robot_id,
                update_count,
                free_cells,
                unknown_cells,
                start_log_odds if start_log_odds is not None else float("nan"),
            )
        # Frontier count is available in explorer_bot logs; avoid duplicate spam here.

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

                if self._is_likely_free(neighbor, map_data):
                    queue.append(neighbor)
                elif map_data.is_voxel_unknown(neighbor):
                    # Check if this unknown cell has at least one free neighbor
                    neighbor_neighbors = map_data.get_voxel_neighbors(neighbor, connectivity=4)
                    has_free_neighbor = any(
                        self._is_likely_free(n, map_data) for n in neighbor_neighbors
                    )
                    
                    if has_free_neighbor:
                        # Found a frontier cell, now find the entire frontier region
                        frontier_region = self.find_frontier_region(neighbor, map_data)
                        if frontier_region and frontier_region.size > 0:  # Only add non-empty regions
                            frontiers.append(frontier_region)

        return frontiers

    def find_frontier_region(self, start_cell, map_data: OccupancyGrid2d):
        """
        Implementation of Algorithm 2 - Frontier region connectivity search

        Finds a complete frontier region starting from a frontier cell.
        A frontier region is a connected set of unknown cells that each have at least one free neighbor.
        """
        region = Frontier()
        queue = [start_cell]
        visited = set()

        while queue:
            cell = queue.pop(0)

            if cell in visited:
                continue

            visited.add(cell)

            # Check if this cell has at least one free neighbor
            neighbors = map_data.get_voxel_neighbors(cell, connectivity=4)
            has_free_neighbor = any(self._is_likely_free(n, map_data) for n in neighbors)
            
            if has_free_neighbor:
                region.add_cell(cell)

                # Add unknown neighbors to the queue
                for neighbor in neighbors:
                    if map_data.is_voxel_unknown(neighbor) and neighbor not in visited:
                        queue.append(neighbor)

        return region if region.size > 0 else None

    def find_frontiers_global(self, map_data: OccupancyGrid2d):
        """Fallback global frontier extraction across the whole map."""
        frontiers = []
        visited_unknown = set()
        for ii in range(map_data._x_num):
            for jj in range(map_data._y_num):
                cell = (ii, jj)
                if cell in visited_unknown:
                    continue
                if not self._is_frontier_cell(cell, map_data):
                    continue

                region = Frontier()
                queue = [cell]
                while queue:
                    curr = queue.pop(0)
                    if curr in visited_unknown:
                        continue
                    visited_unknown.add(curr)
                    if not self._is_frontier_cell(curr, map_data):
                        continue
                    region.add_cell(curr)
                    for neighbor in map_data.get_voxel_neighbors(curr, connectivity=4):
                        if neighbor not in visited_unknown and map_data.is_voxel_unknown(neighbor):
                            queue.append(neighbor)
                if region.size > 0:
                    frontiers.append(region)
        return frontiers

    

    def get_best_frontier(self, point):
        """
        Select a frontier by cost using world-frame distance in **meters** to the
        nearest frontier cell center (consistent with ``collision_radius`` etc.).
        Args:
            point: ``(x, y)`` in map / world meters
        Returns:
            Preferred ``Frontier`` region or ``None``
        """
        wx, wy = float(point[0]), float(point[1])
        eligible_frontiers = [
            f for f in self.frontiers if f.size >= self.frontier_min_region_size
        ]
        if not eligible_frontiers:
            eligible_frontiers = self.frontiers

        closest_frontier = None
        min_cost = float("inf")

        for frontier in eligible_frontiers:
            distance_m = frontier.min_world_distance_to(wx, wy, self.occupancy_map)
            # Saturated size bonus in "meters of virtual distance reduction".
            # log1p() prevents giant regions from dominating selection.
            size_bonus_m = (
                self.frontier_size_bonus_scale_m
                * max(self.frontier_size_wt, 0.0)
                * np.log1p(float(frontier.size))
            )
            cost = self.frontier_dist_wt * distance_m - size_bonus_m
            if cost < min_cost:
                min_cost = cost
                closest_frontier = frontier

        return closest_frontier

    def frontier_to_world_point(self, frontier, reference_world_point=None):
        """
        Convert a frontier region to a concrete world target point.
        If ``reference_world_point`` is provided, pick the frontier cell nearest
        to that point (more stable than centroids that can fall in explored space).
        """
        if frontier is None:
            return None
        if not frontier.cells:
            return None

        if reference_world_point is not None:
            wx, wy = float(reference_world_point[0]), float(reference_world_point[1])
            best_cell = None
            best_dist = float("inf")
            for cell in frontier.cells:
                ii = int(round(cell[0]))
                jj = int(round(cell[1]))
                cx, cy = self.occupancy_map.get_voxel_center(ii, jj)
                d = float(np.hypot(wx - cx, wy - cy))
                if d < best_dist:
                    best_dist = d
                    best_cell = (ii, jj)
            if best_cell is not None:
                return self._frontier_approach_point(best_cell, wx, wy)

        # Fallback: centroid if no reference point is available.
        centroid = frontier.get_centroid()
        ii = int(round(centroid[0]))
        jj = int(round(centroid[1]))
        if reference_world_point is not None:
            wx, wy = float(reference_world_point[0]), float(reference_world_point[1])
        else:
            wx, wy = self.occupancy_map.get_voxel_center(ii, jj)
        return self._frontier_approach_point((ii, jj), wx, wy)

    def _frontier_approach_point(self, frontier_cell, wx, wy):
        """
        Return a world target near a frontier cell but in traversable free space.
        Choosing unknown-cell centers directly can cause orbiting at the boundary.
        """
        neighbors = self.occupancy_map.get_voxel_neighbors(frontier_cell, connectivity=4)
        free_neighbors = [n for n in neighbors if self._is_likely_free(n, self.occupancy_map)]
        if not free_neighbors:
            return self.occupancy_map.get_voxel_center(frontier_cell[0], frontier_cell[1])

        best_neighbor = None
        best_dist = float("inf")
        for n in free_neighbors:
            cx, cy = self.occupancy_map.get_voxel_center(n[0], n[1])
            d = float(np.hypot(wx - cx, wy - cy))
            if d < best_dist:
                best_dist = d
                best_neighbor = n

        return self.occupancy_map.get_voxel_center(best_neighbor[0], best_neighbor[1])

    def get_closest_frontier_robot(self, robot_id):
        """
        Get the closest frontier to a given robot.

        Args:
            robot_id: The ID of the robot

        Returns:
            Closest frontier region
  # Flocking parameters
  cohesion_radius: 1.0  # meters
  separation_radius: 0.5 # meters
  alignment_radius: 1.0  # meters
  collision_radius: 0.5  # meters

  # Weights for different behaviors
  cohesion_weight: 0.23
  separation_weight: 1.1
  alignment_weight: 0.5
  obstacle_weight: 1.1
  wall_weight: 1.1
  frontier_weight: 0.08

  # Frontier parameters
  frontier_dist_wt: 0.001
  frontier_size_wt: 1.0

  # Communication parameters
  comm_radius: 5.0     # meters
  max_neighbor_age: 0.5 # seconds 
        """
        # Assuming you have a way to get the robot's position
        robot_position = self.get_robot_position(robot_id)
        cell = self.world_to_grid(robot_position)

        return self.get_closest_frontier(cell)

    def map_fully_known(self):
        """
        Check if the map is fully known.
        """
        return self.occupancy_map.is_fully_known()

    # Helper methods

    def filter_frontiers(self):
        """Filter out small or unreachable frontiers."""
        # Add your filtering logic here
        pass

    def publish_frontiers(self):
        """Publish the detected frontiers to other robots/nodes."""
        # Implementation depends on your messaging system
        pass
