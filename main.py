from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap
import pygame
import argparse
import time

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='D* Lite Path Planning')
    parser.add_argument('--width', type=int, default=800, help='Window width')
    parser.add_argument('--height', type=int, default=600, help='Window height')
    parser.add_argument('--cell-size', type=int, default=40, help='Size of each cell')  # Increased cell size
    parser.add_argument('--margin', type=int, default=1, help='Margin between cells')
    args = parser.parse_args()
    
    # Calculate grid dimensions
    x_dim = args.height // (args.cell_size + args.margin)
    y_dim = args.width // (args.cell_size + args.margin)
    
    # Initialize start and goal positions
    start = (x_dim // 4, y_dim // 4)
    goal = (3 * x_dim // 4, 3 * y_dim // 4)
    
    # Create GUI
    gui = Animation(
        title="D* Lite Path Planning - Press 'T' to change target",
        width=args.cell_size,
        height=args.cell_size,
        margin=args.margin,
        x_dim=x_dim,
        y_dim=y_dim,
        start=start,
        goal=goal,
        viewing_range=5
    )
    
    # Initialize path planner
    dstar = DStarLite(gui.world, start, goal)
    
    # Main loop
    path = None
    running = True
    last_goal = goal
    last_world_state = None
    move_delay = 0.1  # Delay between moves (seconds)
    last_move_time = time.time()
    
    while running:
        current_time = time.time()
        
        # Handle events
        if gui.handle_events():
            running = False
            continue
            
        # Check if goal changed or obstacles changed
        current_world_state = gui.world.occupancy_grid_map.tobytes()
        goal_changed = last_goal != gui.goal
        world_changed = (last_world_state is not None and 
                        current_world_state != last_world_state)
        
        # Reinitialize D* Lite if goal or world changed
        if goal_changed:
            print("Goal changed - Reinitializing path planner")
            dstar = DStarLite(gui.world, gui.current_pos, gui.goal)
            last_goal = gui.goal
            gui.paused = True
            
        # Update last world state
        last_world_state = current_world_state
            
        # Update path if not paused or if changes occurred
        if not gui.paused or goal_changed or world_changed:
            # Compute new path
            new_path, g, rhs = dstar.move_and_replan(gui.current_pos)
            
            if new_path:
                path = new_path
                # Move rover along path with delay
                if current_time - last_move_time >= move_delay and len(path) > 1:
                    gui.current_pos = path[1]  # Move to next position
                    last_move_time = current_time
                    
                if gui.step_mode:
                    gui.paused = True
            else:
                print("No path found - Try removing some obstacles!")
                gui.paused = True
                
        # Render
        gui.render(path)
        gui.clock.tick(60)
        
    pygame.quit()

if __name__ == "__main__":
    main()

