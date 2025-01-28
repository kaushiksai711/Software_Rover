import pygame
import numpy as np
from typing import List, Tuple, Optional
import time
from grid import OccupancyGridMap

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
GRAY = (128, 128, 128)

class Animation:
    def __init__(self, title: str, width: int, height: int, margin: int,
                 x_dim: int, y_dim: int, start: Tuple[int, int],
                 goal: Tuple[int, int], viewing_range: int):
        
        pygame.init()
        pygame.display.set_caption(title)
        
        # Initialize the world map
        self.world = OccupancyGridMap(x_dim=x_dim, y_dim=y_dim)
        
        self.width = width
        self.height = height
        self.margin = margin
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.start = start
        self.current_pos = start  # Add this after self.start = start
        self.goal = goal
        self.viewing_range = viewing_range
        
        # Window size
        window_size = [
            (width + margin) * y_dim + margin,
            (height + margin) * x_dim + margin
        ]
        self.screen = pygame.display.set_mode(window_size)
        
        # Initialize font
        self.font = pygame.font.SysFont('Arial', 16)
        
        # Drawing state
        self.drawing_mode = None  # None, 'obstacle', or 'clear'
        self.last_pos = None
        self.show_grid = True
        self.show_cost = False
        
        # Animation control
        self.paused = True
        self.step_mode = False
        self.done = False
        self.clock = pygame.time.Clock()
        
        # Add new state for goal selection mode
        self.goal_selection_mode = False
        
        # Add rover visualization parameters
        self.rover_radius = min(width, height) // 3
        self.rover_color = BLUE
        self.path_color = (0, 150, 255)  # Light blue
        
    def handle_events(self) -> bool:
        """
        Handle pygame events with improved controls
        Returns True if should quit
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
                
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_g:
                    self.show_grid = not self.show_grid
                elif event.key == pygame.K_c:
                    self.show_cost = not self.show_cost
                elif event.key == pygame.K_s:
                    self.step_mode = not self.step_mode
                elif event.key == pygame.K_t:  # Press 'T' to enter goal selection mode
                    self.goal_selection_mode = not self.goal_selection_mode
                    if self.goal_selection_mode:
                        print("Goal selection mode: ON - Click to set new goal")
                    else:
                        print("Goal selection mode: OFF")
                    
            elif event.type == pygame.MOUSEBUTTONDOWN:
                x = event.pos[1] // (self.height + self.margin)
                y = event.pos[0] // (self.width + self.margin)
                grid_pos = (x, y)
                
                if self.goal_selection_mode and event.button == 1:  # Left click in goal selection mode
                    if self.in_bounds(grid_pos) and grid_pos != self.start:
                        self.goal = grid_pos
                        self.goal_selection_mode = False
                        print(f"New goal set at: {grid_pos}")
                        return False
                
                if not self.goal_selection_mode:  # Normal obstacle drawing mode
                    if event.button == 1:  # Left click
                        self.drawing_mode = 'obstacle'
                        self._handle_mouse_input(event.pos)
                    elif event.button == 3:  # Right click
                        self.drawing_mode = 'clear'
                        self._handle_mouse_input(event.pos)
                    
            elif event.type == pygame.MOUSEBUTTONUP:
                if not self.goal_selection_mode:
                    self.drawing_mode = None
                    self.last_pos = None
                    
            elif event.type == pygame.MOUSEMOTION:
                if not self.goal_selection_mode and self.drawing_mode:
                    self._handle_mouse_input(event.pos)
                    
        return False
        
    def _handle_mouse_input(self, pos: Tuple[int, int]) -> None:
        """
        Handle mouse input for drawing/clearing obstacles
        """
        x = pos[1] // (self.height + self.margin)
        y = pos[0] // (self.width + self.margin)
        
        if not (0 <= x < self.x_dim and 0 <= y < self.y_dim):
            return
            
        current_pos = (x, y)
        
        # Draw line between last position and current position
        if self.last_pos:
            points = self._get_line_points(self.last_pos, current_pos)
            for point in points:
                self._update_cell(point)
        else:
            self._update_cell(current_pos)
            
        self.last_pos = current_pos
        
    def _update_cell(self, pos: Tuple[int, int]) -> None:
        """
        Update cell based on drawing mode
        """
        if pos == self.start or pos == self.goal:
            return
            
        if self.drawing_mode == 'obstacle':
            self.world.set_obstacle(pos)
        elif self.drawing_mode == 'clear':
            self.world.remove_obstacle(pos)
            
    def _get_line_points(self, start: Tuple[int, int], end: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Get points in a line between start and end using Bresenham's algorithm
        """
        points = []
        x1, y1 = start
        x2, y2 = end
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = 1 if x2 > x1 else -1
        sy = 1 if y2 > y1 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x2:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x, y))
        return points
        
    def render(self, path: Optional[List[Tuple[int, int]]] = None) -> None:
        """
        Render the grid with improved visuals
        """
        self.screen.fill(BLACK)
        
        # Draw grid
        for row in range(self.x_dim):
            for col in range(self.y_dim):
                x = (self.margin + self.width) * col + self.margin
                y = (self.margin + self.height) * row + self.margin
                
                # Determine cell color
                if (row, col) == self.start:
                    color = RED
                elif (row, col) == self.goal:
                    color = GREEN
                elif self.world.is_obstacle((row, col)):
                    color = GRAY
                else:
                    color = WHITE
                    
                pygame.draw.rect(self.screen, color,
                               [x, y, self.width, self.height])
        
        # Draw path with gradient color
        if path:
            for i in range(len(path) - 1):
                start_pos = path[i]
                end_pos = path[i + 1]
                
                # Calculate centers of cells
                start_x = (self.margin + self.width) * start_pos[1] + self.margin + self.width // 2
                start_y = (self.margin + self.height) * start_pos[0] + self.margin + self.height // 2
                end_x = (self.margin + self.width) * end_pos[1] + self.margin + self.width // 2
                end_y = (self.margin + self.height) * end_pos[0] + self.margin + self.height // 2
                
                # Draw thicker path line
                pygame.draw.line(self.screen, self.path_color, 
                               (start_x, start_y), (end_x, end_y), 3)
                
                # Draw small circles at path points
                pygame.draw.circle(self.screen, self.path_color, 
                                (start_x, start_y), 4)
                
            # Draw final point
            if path:
                final_pos = path[-1]
                final_x = (self.margin + self.width) * final_pos[1] + self.margin + self.width // 2
                final_y = (self.margin + self.height) * final_pos[0] + self.margin + self.height // 2
                pygame.draw.circle(self.screen, self.path_color, 
                                (final_x, final_y), 4)
        
        # Draw rover at current position
        rover_x = (self.margin + self.width) * self.current_pos[1] + self.margin + self.width // 2
        rover_y = (self.margin + self.height) * self.current_pos[0] + self.margin + self.height // 2
        pygame.draw.circle(self.screen, self.rover_color, 
                         (rover_x, rover_y), self.rover_radius)
        
        # Draw grid lines
        if self.show_grid:
            for x in range(self.x_dim + 1):
                pygame.draw.line(self.screen, GRAY,
                               [0, x * (self.height + self.margin)],
                               [self.y_dim * (self.width + self.margin),
                                x * (self.height + self.margin)], 1)
            for y in range(self.y_dim + 1):
                pygame.draw.line(self.screen, GRAY,
                               [y * (self.width + self.margin), 0],
                               [y * (self.width + self.margin),
                                self.x_dim * (self.height + self.margin)], 1)
        
        # Draw status text
        status_text = f"Press 'T' to change target | Space to start/pause | Left click: Add obstacles | Right click: Remove"
        text_surface = self.font.render(status_text, True, WHITE)
        self.screen.blit(text_surface, (10, 10))
                                
        pygame.display.flip()

    def in_bounds(self, pos: Tuple[int, int]) -> bool:
        """Check if position is within grid bounds"""
        x, y = pos
        return 0 <= x < self.x_dim and 0 <= y < self.y_dim

