import numpy as np
from typing import Tuple, List

def smooth_path(path: List[Tuple[int, int]], weight_data: float = 0.5, weight_smooth: float = 0.1, tolerance: float = 0.000001) -> List[Tuple[int, int]]:
    """
    Implements path smoothing using gradient descent
    """
    if len(path) <= 2:
        return path
        
    newpath = [[x, y] for x, y in path]
    change = tolerance
    
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path)-1):
            for j in range(2):
                old = newpath[i][j]
                newpath[i][j] += weight_data * (path[i][j] - newpath[i][j])
                newpath[i][j] += weight_smooth * (newpath[i-1][j] + newpath[i+1][j] - 2.0 * newpath[i][j])
                change += abs(old - newpath[i][j])
    
    return [(int(x), int(y)) for x, y in newpath]

def improved_heuristic(start: Tuple[int, int], goal: Tuple[int, int]) -> float:
    """
    Enhanced heuristic using diagonal distance
    """
    dx = abs(start[0] - goal[0])
    dy = abs(start[1] - goal[1])
    return max(dx, dy) + 0.414 * min(dx, dy)

