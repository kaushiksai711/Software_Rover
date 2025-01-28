from typing import Dict, List, Tuple, Optional
from priority_queue import PriorityQueue, Priority
from utils import improved_heuristic, smooth_path
import numpy as np

class DStarLite:
    def __init__(self, map, s_start: Tuple[int, int], s_goal: Tuple[int, int]):
        self.map = map
        self.s_start = s_start
        self.s_goal = s_goal
        self.s_last = s_start
        self.k_m = 0
        self.queue = PriorityQueue()
        self.rhs = np.full((map.x_dim, map.y_dim), np.inf)
        self.g = np.full((map.x_dim, map.y_dim), np.inf)
        self.rhs[s_goal] = 0
        self.queue.insert(s_goal, self._calculate_key(s_goal))
        
    def _calculate_key(self, s: Tuple[int, int]) -> Priority:
        """
        Calculate priority for vertex s
        """
        return Priority(
            min(self.g[s], self.rhs[s]) + improved_heuristic(self.s_start, s) + self.k_m,
            min(self.g[s], self.rhs[s])
        )
        
    def _update_vertex(self, u: Tuple[int, int]) -> None:
        """
        Update vertex u and its cost
        """
        if u != self.s_goal:
            min_rhs = float('inf')
            for s in self.map.get_neighbors(u):
                temp = self.g[s] + self._compute_cost(u, s)
                if temp < min_rhs:
                    min_rhs = temp
            self.rhs[u] = min_rhs
            
        if u in self.queue:
            self.queue.remove(u)
            
        if self.g[u] != self.rhs[u]:
            self.queue.insert(u, self._calculate_key(u))
            
    def _compute_cost(self, u: Tuple[int, int], v: Tuple[int, int]) -> float:
        """
        Compute cost between vertices considering diagonal movement and obstacles
        """
        if not self.map.is_valid_move(u, v, True):
            return float('inf')
            
        # Higher cost for moving near obstacles
        obstacle_cost = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                check_pos = (v[0] + dx, v[1] + dy)
                if self.map.in_bounds(check_pos) and self.map.is_obstacle(check_pos):
                    obstacle_cost += 2.0

        # Base cost (diagonal movement costs more)
        if abs(u[0] - v[0]) == 1 and abs(u[1] - v[1]) == 1:
            base_cost = 1.414  # √2
        else:
            base_cost = 1.0

        return base_cost + obstacle_cost
        
    def compute_shortest_path(self) -> None:
        """
        Compute shortest path with improved termination condition
        """
        while (self.queue and 
              (self.queue.top_key() < self._calculate_key(self.s_start) or 
               self.rhs[self.s_start] != self.g[self.s_start])):
            
            k_old = self.queue.top_key()
            u = self.queue.pop()
            
            if k_old < self._calculate_key(u):
                self.queue.insert(u, self._calculate_key(u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.map.get_neighbors(u):
                    self._update_vertex(s)
            else:
                self.g[u] = float('inf')
                for s in self.map.get_neighbors(u) + [u]:
                    self._update_vertex(s)
                    
    def get_path(self, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Extract path from current position to goal
        """
        if self.g[current] == float('inf'):
            return []
            
        path = [current]
        while current != self.s_goal:
            neighbors = self.map.get_neighbors(current)
            min_cost = float('inf')
            next_pos = None
            
            for neighbor in neighbors:
                cost = self._compute_cost(current, neighbor) + self.g[neighbor]
                if cost < min_cost:
                    min_cost = cost
                    next_pos = neighbor
                    
            if next_pos is None:
                return []
                
            current = next_pos
            path.append(current)
            
        return path
                    
    def move_and_replan(self, robot_position: Tuple[int, int]) -> Tuple[List[Tuple[int, int]], np.ndarray, np.ndarray]:
        """
        Move robot and replan path with smoothing
        """
        self.s_start = robot_position
        
        # Compute shortest path
        self.compute_shortest_path()
        
        # Get complete path to goal
        path = self.get_path(robot_position)
        
        if not path:
            return [], self.g, self.rhs
            
        # Smooth the path
        smoothed_path = smooth_path(path)
        return smoothed_path, self.g, self.rhs

