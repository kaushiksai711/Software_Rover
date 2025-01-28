import numpy as np
from typing import List, Tuple, Optional
from utils import improved_heuristic

class OccupancyGridMap:
    def __init__(self, x_dim: int, y_dim: int, exploration_setting: str = '8N'):
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.exploration_setting = exploration_setting
        self.occupancy_grid_map = np.zeros((x_dim, y_dim), dtype=np.uint8)
        self.obstacle_threshold = 0.7  # Probability threshold for obstacle detection
        
    def update_cell(self, pos: Tuple[int, int], value: int) -> None:
        """
        Update cell with probability of being obstacle
        """
        if self.in_bounds(pos):
            x, y = pos
            current = self.occupancy_grid_map[x, y]
            # Using log odds update
            log_odds = np.log(value / (1.0 - value))
            self.occupancy_grid_map[x, y] = min(255, max(0, current + log_odds))
            
    def get_neighbors(self, pos: Tuple[int, int], avoid_obstacles: bool = True) -> List[Tuple[int, int]]:
        """
        Get valid neighbors considering diagonal movements
        """
        x, y = pos
        neighbors = []
        
        # All possible movements (8-directional)
        movements = [
            (x-1, y), (x+1, y), (x, y-1), (x, y+1),  # Cardinal directions
            (x-1, y-1), (x-1, y+1), (x+1, y-1), (x+1, y+1)  # Diagonal directions
        ]
        
        for next_pos in movements:
            if self.is_valid_move(pos, next_pos, avoid_obstacles):
                neighbors.append(next_pos)
                
        return neighbors
        
    def is_valid_move(self, current: Tuple[int, int], next_pos: Tuple[int, int], avoid_obstacles: bool) -> bool:
        """
        Check if move is valid considering diagonal obstacles
        """
        if not self.in_bounds(next_pos):
            return False
            
        if avoid_obstacles and not self.is_unoccupied(next_pos):
            return False
            
        # Check diagonal movement
        if abs(current[0] - next_pos[0]) == 1 and abs(current[1] - next_pos[1]) == 1:
            # Check if both adjacent cells are obstacles (can't squeeze through)
            if not self.is_unoccupied((current[0], next_pos[1])) and \
               not self.is_unoccupied((next_pos[0], current[1])):
                return False
                
        return True

import numpy as np
from typing import List, Tuple, Optional
from utils import improved_heuristic

class OccupancyGridMap:
    def __init__(self, x_dim: int, y_dim: int, exploration_setting: str = '8N'):
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.exploration_setting = exploration_setting
        self.occupancy_grid_map = np.zeros((x_dim, y_dim), dtype=np.uint8)
        self.obstacle_threshold = 0.7  # Probability threshold for obstacle detection
        
    def update_cell(self, pos: Tuple[int, int], value: int) -> None:
        """
        Update cell with probability of being obstacle
        """
        if self.in_bounds(pos):
            x, y = pos
            current = self.occupancy_grid_map[x, y]
            # Using log odds update
            log_odds = np.log(value / (1.0 - value))
            self.occupancy_grid_map[x, y] = min(255, max(0, current + log_odds))
            
    def get_neighbors(self, pos: Tuple[int, int], avoid_obstacles: bool = True) -> List[Tuple[int, int]]:
        """
        Get valid neighbors considering diagonal movements
        """
        x, y = pos
        neighbors = []
        
        # All possible movements (8-directional)
        movements = [
            (x-1, y), (x+1, y), (x, y-1), (x, y+1),  # Cardinal directions
            (x-1, y-1), (x-1, y+1), (x+1, y-1), (x+1, y+1)  # Diagonal directions
        ]
        
        for next_pos in movements:
            if self.is_valid_move(pos, next_pos, avoid_obstacles):
                neighbors.append(next_pos)
                
        return neighbors
        
    def is_valid_move(self, current: Tuple[int, int], next_pos: Tuple[int, int], avoid_obstacles: bool) -> bool:
        """
        Check if move is valid considering diagonal obstacles
        """
        if not self.in_bounds(next_pos):
            return False
            
        if avoid_obstacles and not self.is_unoccupied(next_pos):
            return False
            
        # Check diagonal movement
        if abs(current[0] - next_pos[0]) == 1 and abs(current[1] - next_pos[1]) == 1:
            # Check if both adjacent cells are obstacles (can't squeeze through)
            if not self.is_unoccupied((current[0], next_pos[1])) and \
               not self.is_unoccupied((next_pos[0], current[1])):
                return False
                
        return True

    def set_obstacle(self, pos: Tuple[int, int]) -> None:
        """Set a cell as an obstacle"""
        if self.in_bounds(pos):
            x, y = pos
            self.occupancy_grid_map[x, y] = 255

    def remove_obstacle(self, pos: Tuple[int, int]) -> None:
        """Remove an obstacle from a cell"""
        if self.in_bounds(pos):
            x, y = pos
            self.occupancy_grid_map[x, y] = 0

    def is_obstacle(self, pos: Tuple[int, int]) -> bool:
        """Check if a cell contains an obstacle"""
        if self.in_bounds(pos):
            x, y = pos
            return self.occupancy_grid_map[x, y] == 255
        return False

    def in_bounds(self, pos: Tuple[int, int]) -> bool:
        x, y = pos
        return 0 <= x < self.x_dim and 0 <= y < self.y_dim

    def is_unoccupied(self, pos: Tuple[int, int]) -> bool:
        if self.in_bounds(pos):
            x, y = pos
            return self.occupancy_grid_map[x, y] < self.obstacle_threshold
        return False

