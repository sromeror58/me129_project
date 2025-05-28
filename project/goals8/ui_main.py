import threading
import numpy as np
import copy

class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        
        # Mode flags
        self.mode = 4  # 0=manual, 1=explore, 2=goal, 3=paused, 4=uninitialized. -1=quit
        self.take_step = False  # True when in step-by-step mode
        # Robot state
        self.pose = None  # Current pose (x, y, heading)
        self.goal = None  # Current goal coordinates (x, y)
        # Command state
        self.command = None  # Current command to execute
        # Map state
        self.map_filename = None  # Current map filename for save/load operations
        self.before_pause = None
        # ROS parameters
        self.robotx = 0 # Copy of the robot’s x
        self.roboty = 0 # Copy of the robot’s y
        self.robotheading = 0 # Copy of the robot’s heading

    def acquire(self):
        return self.lock.acquire()

    def release(self):
        return self.lock.release()

def runui(shared):
    """
    UI thread function that handles user input and updates the shared mode.
    Args:
        shared: SharedData object containing the robot state and lock
    """

    while shared.mode==4:
        continue

    print("\nRobot Control UI")
    print("Available commands:\n")
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
                        print("Starting autonomous exploration")

                    elif command == 'clear':
                        shared.command = 'clear'
                        print("Clearing all blockages")

                    elif command == 'goal':
                        try:
                            x = int(input("Enter goal x-coordinate: "))
                            y = int(input("Enter goal y-coordinate: "))
                            shared.mode = 2  # Goal mode
                            shared.goal = copy.deepcopy((x, y))
                            print(f"Setting goal to ({x}, {y})")
                        except ValueError:
                            print("Invalid coordinates. Please enter integers.")

                    elif command == 'pause':
                        if shared.mode in [1, 2]:  # Only pause if in explore or goal mode
                            shared.before_pause = shared.mode
                            shared.mode = 3  # Paused mode
                            print("Pausing autonomous movement")
                        else:
                            print("No autonomous movement to pause")

                    elif command == 'step':
                        if shared.mode == 3:  # Only step if paused
                            shared.take_step = True
                            print("Taking one step")
                        else:
                            print("Must be paused to use step command")

                    elif command == 'resume':
                        if shared.mode == 3:  # Only resume if paused
                            shared.mode = 1 if shared.goal is None else 2
                            print("Resuming autonomous movement")
                        else:
                            print("Not in paused mode")

                    elif command == 'show':
                        print(f"Updating map visualization")
                        shared.command = command

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
                                shared.pose = copy.deepcopy((x, y, heading))
                                shared.command = 'pose'
                                print(f"Setting pose to ({x}, {y}, {heading})")
                            else:
                                print("Invalid heading. Must be 0-7.")
                        except ValueError:
                            print("Invalid input. Please enter integers.")

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
