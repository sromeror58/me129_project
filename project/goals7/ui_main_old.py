import threading
import time
import matplotlib
matplotlib.use('TkAgg')  # Ensure we use TkAgg backend for GUI display
import matplotlib.pyplot as plt
from map import Map, STATUS
from config import DX_DY_TABLE
import math

class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        
        # Mode flags
        self.mode = 0  # 0=manual, 1=explore, 2=goal, 3=paused
        self.step_mode = False  # True when in step-by-step mode
        
        # Robot state
        self.pose = None  # Current pose (x, y, heading)
        self.goal = None  # Current goal coordinates (x, y)
        self.num_streets_to_goal = [0, []]  # [turn_direction, [streets_to_encounter]]
        
        # Command state
        self.command = None  # Current command to execute
        self.command_params = {}  # Additional parameters for commands
        
        # Map state
        self.map_filename = None  # Current map filename for save/load operations
        self.map = None  # Reference to the Map object

    def acquire(self):
        return self.lock.acquire()

    def release(self):
        return self.lock.release()

def plot_map(shared):
    """
    Thread-safe function to plot the current map state.
    This function should only be called from the UI thread.
    """
    if not shared.acquire():
        return
    
    try:
        if shared.map is None or shared.pose is None:
            print("Cannot show map: Map or pose not initialized")
            return
            
        # Clear the current figure
        plt.clf()
        
        # Create a new axes, enable the grid, and set axis limits
        plt.axes()
        plt.gca().set_xlim(-3.5, 3.5)
        plt.gca().set_ylim(-3.5, 3.5)
        plt.gca().set_aspect("equal")
        
        # Show all possible locations
        for x_grid in range(-3, 4):
            for y_grid in range(-3, 4):
                plt.plot(x_grid, y_grid, color="lightgray", marker="o", markersize=8)
        
        # Get the direction vector for the current heading
        dx, dy = DX_DY_TABLE[shared.pose[2]]
        
        # Scale the direction vector to length 0.5
        arrow_length = 0.5
        scale = arrow_length / math.sqrt(dx * dx + dy * dy)
        dx *= scale
        dy *= scale
        
        # Calculate start and end points to center the arrow
        x_start = shared.pose[0] - dx / 2
        y_start = shared.pose[1] - dy / 2
        
        # Draw the robot arrow
        plt.arrow(
            x_start, y_start,
            dx, dy,
            width=0.2,
            head_width=0.3,
            head_length=0.1,
            color="magenta"
        )
        
        # Define colors for each status
        status_colors = {
            STATUS.UNKNOWN: "black",
            STATUS.NONEXISTENT: "lightgray",
            STATUS.UNEXPLORED: "blue",
            STATUS.DEADEND: "red",
            STATUS.CONNECTED: "green"
        }
        
        # Draw the intersections and streets
        for x_int, y_int in shared.map.intersections:
            for h in range(8):
                dx, dy = DX_DY_TABLE[h]
                dx *= 0.5
                dy *= 0.5
                
                status = shared.map.getintersection(x_int, y_int).streets[h]
                color = status_colors[status]
                
                plt.plot([x_int, x_int + dx], [y_int, y_int + dy], color=color)
        
        # Update the display
        plt.draw()
        plt.pause(0.1)
        
    finally:
        shared.release()

def runui(shared):
    """
    UI thread function that handles user input and updates the shared mode.
    Args:
        shared: SharedData object containing the robot state and lock
    """
    print("\nRobot Control UI")
    print("Available commands:")
    print("  'explore'   - Start autonomous exploration")
    print("  'goal'      - Drive to specified coordinates")
    print("  'pause'     - Pause autonomous movement")
    print("  'step'      - Take one action (when paused)")
    print("  'resume'    - Resume autonomous movement")
    print("  'left'      - Take one left turn")
    print("  'right'     - Take one right turn")
    print("  'straight'  - Drive straight to next intersection")
    print("  'save'      - Save current map")
    print("  'load'      - Load map from file")
    print("  'pose'      - Set robot position/heading")
    print("  'show'      - Visualize current map")
    print("  'quit'      - Exit program")
    
    while True:
        try:
            # Grab the user input - implicitly sleep while waiting
            command = input("\nEnter command: ").strip().lower()
            
            # Grab access to the shared data
            if shared.acquire():
                try:
                    # Process the command
                    if command == 'explore':
                        shared.mode = 1  # Explore mode
                        shared.goal = None
                        shared.step_mode = False
                        print("Starting autonomous exploration")

                    elif command == 'goal':
                        try:
                            x = int(input("Enter goal x-coordinate: "))
                            y = int(input("Enter goal y-coordinate: "))
                            shared.mode = 2  # Goal mode
                            shared.goal = (x, y)
                            shared.step_mode = False
                            print(f"Setting goal to ({x}, {y})")
                        except ValueError:
                            print("Invalid coordinates. Please enter integers.")

                    elif command == 'pause':
                        if shared.mode in [1, 2]:  # Only pause if in explore or goal mode
                            shared.mode = 3  # Paused mode
                            print("Pausing autonomous movement")
                        else:
                            print("No autonomous movement to pause")

                    elif command == 'step':
                        if shared.mode == 3:  # Only step if paused
                            shared.step_mode = True
                            print("Taking one step")
                        else:
                            print("Must be paused to use step command")

                    elif command == 'resume':
                        if shared.mode == 3:  # Only resume if paused
                            shared.mode = 1 if shared.goal is None else 2
                            shared.step_mode = False
                            print("Resuming autonomous movement")
                        else:
                            print("Not in paused mode")

                    elif command in ['left', 'right', 'straight']:
                        shared.mode = 0  # Manual mode
                        shared.command = command
                        print(f"Executing {command} movement")

                    elif command == 'save':
                        shared.map_filename = input("Enter filename to save map: ")
                        shared.command = 'save'
                        print(f"Saving map to {shared.map_filename}")

                    elif command == 'load':
                        shared.map_filename = input("Enter filename to load map: ")
                        shared.command = 'load'
                        print(f"Loading map from {shared.map_filename}")

                    elif command == 'pose':
                        try:
                            x = int(input("Enter x-coordinate: "))
                            y = int(input("Enter y-coordinate: "))
                            heading = int(input("Enter heading (0-7): "))
                            if 0 <= heading <= 7:
                                shared.pose = (x, y, heading)
                                shared.command = 'pose'
                                print(f"Setting pose to ({x}, {y}, {heading})")
                            else:
                                print("Invalid heading. Must be 0-7.")
                        except ValueError:
                            print("Invalid input. Please enter integers.")

                    elif command == 'show':
                        print("Updating map visualization")
                        # Release the lock before plotting to avoid deadlock
                        shared.release()
                        plot_map(shared)
                        # Re-acquire the lock for the next iteration
                        if not shared.acquire():
                            print("Failed to re-acquire lock after plotting")
                            break
                        continue  # Skip the release at the end of the loop

                    elif command == 'quit':
                        shared.mode = -1
                        print("Exiting program...")
                        break

                    else:
                        print("Unknown command. Please use one of the available commands.")

                finally:
                    # Always release the lock
                    shared.release()
            
        except KeyboardInterrupt:
            if shared.acquire():
                shared.mode = -1
                shared.release()
            print("\nExiting due to keyboard interrupt...")
            break

        except Exception as e:
            print(f"Error in UI thread: {e}")
            continue
