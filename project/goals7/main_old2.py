import pigpio
import traceback
import os
import argparse
from drive_system import DriveSystem
from sensor import LineSensor
from behaviors import Behaviors
from pose import Pose
from magnetometer import ADC
from map import Map, Intersection, STATUS
import time
import threading
from ui_main import SharedData, runui


class Robot:
    """
    Class that encapsulates all the robot components such as the drive system
    and sensors.

    This class serves as a high-level interface to control the robot's hardware
    components, providing methods to initialize and safely shut down the robot.
    """

    def __init__(self, io):
        """
        Initializes the motors, drive system and sensors.

        Args:
            io (pigpio.pi): pigpio interface instance for hardware communication
        """
        self.io = io
        self.drive_system = DriveSystem(io)
        self.sensors = LineSensor(io)
        # ADC is initialized separately in main for Behaviors

    def stop(self):
        """
        Safely stop all motors and clean up the pigpio connection.

        This method ensures that all motors are stopped and the pigpio interface
        is properly closed, even if exceptions occur during the shutdown process.
        """
        try:
            # First stop the motors through the drive system
            self.drive_system.stop()
        except Exception as e:
            print(f"Error stopping drive system: {e}")

        try:
            # Then cleanly stop the pigpio interface if still connected
            if hasattr(self.io, "connected") and self.io.connected:
                self.io.stop()
        except Exception as e:
            print(f"Error stopping pigpio: {e}")


# The old initialize_map and initialization_helper are superseded by UI commands 
# and direct initialization in runrobot.

def runrobot(shared: SharedData, behaviors: Behaviors, robot_hardware: Robot, initial_x: float, initial_y: float, initial_heading: int):
    """
    Main robot control loop, running in the main thread.
    It interacts with the UI thread via the shared SharedData object.
    """
    print(f"Robot thread: Initializing with pose ({initial_x}, {initial_y}, {initial_heading})")
    current_pose = Pose(initial_x, initial_y, initial_heading)
    
    initial_streets = [STATUS.UNKNOWN] * 8
    map_obj = Map(current_pose, {(current_pose.x, current_pose.y): Intersection(current_pose.x, current_pose.y, initial_streets)})
    map_obj.plot(current_pose)

    num_streets_to_goal = [0, []] 
    running = True
    
    # To store the mode before pausing, to allow resume to correct mode
    previous_autonomous_mode_before_pause = 1 # Default to explore if paused without prior auto mode

    try:
        while running:
            local_mode = 0
            local_goal = None
            local_command = None
            local_map_filename = None
            local_ui_pose_update = None
            local_step_mode = False
            
            if shared.acquire():
                try:
                    local_mode = shared.mode
                    local_goal = shared.goal
                    local_command = shared.command
                    if local_command: # If there is a command, consume it
                        shared.command = None
                    
                    local_map_filename = shared.map_filename
                    # map_filename is consumed after use by specific commands

                    local_ui_pose_update = shared.pose # UI might have set this
                    
                    local_step_mode = shared.step_mode
                    # step_mode is consumed by this thread when a step is taken

                    # Update shared.pose with the robot's current position for UI
                    shared.pose = (current_pose.x, current_pose.y, current_pose.heading)
                finally:
                    shared.release()

            if local_mode == -1:
                print("Robot thread: Quit command received from UI.")
                running = False
                continue

            # --- Process High-Priority UI Commands (state changes, no immediate movement) ---
            if local_command == 'load':
                if local_map_filename:
                    try:
                        print(f"Robot thread: Loading map from {local_map_filename}.pickle")
                        map_obj = Map.load_map(local_map_filename)
                        # Important: After loading, current_pose might be invalid for the new map.
                        # The user *must* use the 'pose' command to set a valid starting point.
                        print(f"Map '{local_map_filename}' loaded. Use 'pose' command to set robot's position on this map.")
                        # We can't assume a starting pose from the loaded map without a convention.
                        # Plotting with current_pose might be misleading until 'pose' is used.
                        map_obj.plot(current_pose) # Plot with potentially old pose, user needs to update
                    except FileNotFoundError:
                        print(f"Robot thread: Error loading map. File '{local_map_filename}.pickle' not found.")
                    except Exception as e:
                        print(f"Robot thread: Error loading map '{local_map_filename}': {e}")
                    if shared.acquire(): # Consume map_filename
                        shared.map_filename = None
                        shared.release()
                local_command = None # Command processed

            elif local_command == 'save':
                if local_map_filename:
                    print(f"Robot thread: Saving map to {local_map_filename}.pickle")
                    map_obj.save_map(filename=local_map_filename)
                    if shared.acquire(): # Consume map_filename
                        shared.map_filename = None
                        shared.release()
                local_command = None

            elif local_command == 'pose':
                if local_ui_pose_update: # This was set by the UI via shared.pose
                    print(f"Robot thread: Setting pose to {local_ui_pose_update}")
                    current_pose.x = local_ui_pose_update[0]
                    current_pose.y = local_ui_pose_update[1]
                    current_pose.heading = local_ui_pose_update[2]
                    if not map_obj.getintersection(current_pose.x, current_pose.y):
                        new_streets = [STATUS.UNKNOWN] * 8
                        map_obj.intersections[(current_pose.x, current_pose.y)] = Intersection(current_pose.x, current_pose.y, new_streets)
                        print(f"Robot thread: Added new intersection at ({current_pose.x},{current_pose.y}) due to 'pose' command.")
                    map_obj.plot(current_pose)
                local_command = None

            elif local_command == 'show':
                print("Robot thread: Updating map visualization.")
                map_obj.plot(current_pose)
                local_command = None
            
            # --- Determine `action_cmd` (robot's next physical action: 's', 'l', 'r', or None) ---
            action_cmd = None
            current_intersection = map_obj.getintersection(current_pose.x, current_pose.y)
            if not current_intersection:
                print(f"CRITICAL ERROR: Robot at ({current_pose.x},{current_pose.y}) which is not a known intersection! Check map or use 'pose'. Switching to manual.")
                if shared.acquire():
                    shared.mode = 0
                    shared.release()
                local_mode = 0 

            # --- Main decision logic for action_cmd ---
            # This section decides what action_cmd should be, based on current mode
            # It does NOT yet consider if mode is PAUSED and not stepping. That's handled later.
            
            if local_mode == 0: # Manual
                if local_command in ['left', 'right', 'straight']:
                    action_cmd = local_command[0]
                    print(f"Robot thread: Manual action: {action_cmd}")

            elif local_mode == 1: # Explore
                previous_autonomous_mode_before_pause = 1
                unexplored_streets_at_current = map_obj.get_unexplored_streets(current_pose.x, current_pose.y)
                
                target_heading_for_explore = None
                if unexplored_streets_at_current:
                    # Prefer to explore from current intersection first
                    # Choose closest unexplored street (logic from original main.py)
                    closest_street_info = min(unexplored_streets_at_current, key=lambda x_info: min((x_info[0] - current_pose.heading) % 8, (current_pose.heading - x_info[0]) % 8))
                    target_heading_for_explore = closest_street_info[0]
                    print(f"Robot thread: Exploring - Target heading at current intersection: {target_heading_for_explore}")
                else:
                    # No unexplored streets here, find nearest intersection with unexplored streets
                    nearest_unexplored_coord = map_obj.find_nearest_unexplored(current_pose.x, current_pose.y)
                    if nearest_unexplored_coord is None:
                        print("Robot thread: Map fully explored! Switching to manual mode.")
                        if shared.acquire():
                            shared.mode = 0
                            shared.release()
                        local_mode = 0 # Update local_mode to prevent action_cmd
                        action_cmd = None
                    else:
                        print(f"Robot thread: Exploring - Pathing to nearest unexplored intersection: {nearest_unexplored_coord}")
                        map_obj.setstreet(nearest_unexplored_coord[0], nearest_unexplored_coord[1])
                        current_intersection = map_obj.getintersection(current_pose.x, current_pose.y) # Re-fetch with direction
                        if current_intersection and current_intersection.direction is not None:
                            target_heading_for_explore = current_intersection.direction
                        else:
                            print(f"Robot thread: Exploring - No path to {nearest_unexplored_coord}. Stuck? Switching to manual.")
                            if shared.acquire(): shared.mode = 0; shared.release()
                            local_mode = 0; action_cmd = None
                
                if target_heading_for_explore is not None and local_mode == 1: # Check local_mode again in case it changed
                    if current_pose.heading == target_heading_for_explore:
                        action_cmd = 's'
                    else:
                        heading_diff = (target_heading_for_explore - current_pose.heading) % 8
                        action_cmd = 'l' if heading_diff <= 4 else 'r'
                    num_streets_to_goal = [0, []] # Exploration turns are simple, not using num_streets_to_goal path

            elif local_mode == 2: # Goal
                previous_autonomous_mode_before_pause = 2
                if local_goal is None:
                    print("Robot thread: Goal mode, but no goal set. Switching to manual.")
                    if shared.acquire(): shared.mode = 0; shared.release()
                    local_mode = 0
                elif (current_pose.x, current_pose.y) == local_goal:
                    print("Robot thread: Goal reached! Switching to manual mode.")
                    if shared.acquire(): shared.mode = 0; shared.goal = None; shared.release()
                    local_mode = 0
                    map_obj.setstreet(None, None) # Clear optimal path tree
                    num_streets_to_goal = [0, []]
                    map_obj.plot(current_pose) # Show final position at goal
                elif current_intersection: # Ensure current_intersection is valid
                    map_obj.setstreet(local_goal[0], local_goal[1])
                    current_intersection = map_obj.getintersection(current_pose.x, current_pose.y) # Re-fetch with direction
                    if current_intersection and current_intersection.direction is not None:
                        if current_pose.heading == current_intersection.direction:
                            action_cmd = 's'
                            print(f"Robot thread: Goal Mode - Going straight towards {local_goal}")
                        else: # Need to turn
                            if not num_streets_to_goal[1]: # Calculate turn path if not already doing so
                                heading_diff = (current_intersection.direction - current_pose.heading) % 8
                                num_streets_to_goal[0] = 1 if heading_diff <= 4 else -1 # 1 for left, -1 for right
                                
                                temp_h = current_pose.heading
                                streets_list = []
                                while temp_h != current_intersection.direction:
                                    temp_h = (temp_h + num_streets_to_goal[0]) % 8
                                    if current_intersection.streets[temp_h] != STATUS.NONEXISTENT:
                                        streets_list.append(temp_h)
                                num_streets_to_goal[1] = streets_list
                                print(f"Robot thread: Goal Mode - Calculated turn path: {num_streets_to_goal}")

                            action_cmd = 'l' if num_streets_to_goal[0] > 0 else 'r'
                            print(f"Robot thread: Goal Mode - Turning {action_cmd} towards {current_intersection.direction}. Path: {num_streets_to_goal[1]}")
                    else:
                        print(f"Robot thread: Goal Mode - No valid path to goal {local_goal}. Switching to manual.")
                        if shared.acquire(): shared.mode = 0; shared.goal = None; shared.release()
                        local_mode = 0
                        map_obj.setstreet(None, None)
                        num_streets_to_goal = [0, []]

            # --- Handle Pause and Step ---
            if local_mode == 3: # Paused
                print("Robot thread: Paused.")
                if local_step_mode:
                    print("Robot thread: Step command active.")
                    # Action_cmd should have been determined by the underlying auto mode (explore/goal)
                    # Now, consume the step_mode flag
                    if shared.acquire():
                        shared.step_mode = False 
                        shared.release()
                    # action_cmd determined by previous block (explore/goal) will be used
                    if action_cmd:
                         print(f"Robot thread: Stepping with action: {action_cmd}")
                    else:
                         print("Robot thread: Step requested, but no autonomous action determined (e.g. goal reached or explore stuck).")
                else: # Paused and not stepping
                    action_cmd = None # Override any decided action, robot waits
            
            # --- Execute Physical Action ---
            if action_cmd:
                print(f"Robot thread: Executing physical action: {action_cmd}")
                pose0 = current_pose.clone()
                
                # Pre-check for straight action against NONEXISTENT street
                if action_cmd == 's':
                    current_int_for_straight = map_obj.getintersection(current_pose.x, current_pose.y)
                    if current_int_for_straight and current_int_for_straight.streets[current_pose.heading] == STATUS.NONEXISTENT:
                        print(f"Robot thread: Cannot go straight from ({current_pose.x},{current_pose.y}) heading {current_pose.heading}, street is NONEXISTENT.")
                        action_cmd = None # Cancel action

                if action_cmd == 'l':
                    turnAngle, _ = behaviors.turn_to_next_street("left")
                    if not num_streets_to_goal[1]: # Simple turn or exploration
                        current_pose.calcturn(turnAngle, True)
                    else: # Accurate turn leg for goal mode
                        current_pose.heading = num_streets_to_goal[1].pop(0)
                        if not num_streets_to_goal[1]: num_streets_to_goal[0] = 0 # Turn sequence complete
                    # Add heading correction logic here if Pose.calcturn isn't robust enough
                    map_obj.outcomeA(pose0, current_pose, True)

                elif action_cmd == 'r':
                    turnAngle, _ = behaviors.turn_to_next_street("right")
                    if not num_streets_to_goal[1]:
                        current_pose.calcturn(turnAngle, False)
                    else:
                        current_pose.heading = num_streets_to_goal[1].pop(0)
                        if not num_streets_to_goal[1]: num_streets_to_goal[0] = 0
                    # Add heading correction logic here
                    map_obj.outcomeA(pose0, current_pose, False)

                elif action_cmd == 's':
                    isUturn, _, road_ahead = behaviors.line_follow()
                    if not isUturn:
                        current_pose.calcmove()
                        map_obj.outcomeB(pose0, current_pose, road_ahead)
                    else:
                        current_pose.calcuturn()
                        map_obj.outcomeC(pose0, current_pose, road_ahead)
                
                if action_cmd: # If an action was actually performed
                    map_obj.plot(current_pose)
            
            time.sleep(0.1) # Loop delay

    except KeyboardInterrupt:
        print("Robot thread: Keyboard interrupt. Exiting runrobot.")
    except Exception as e:
        print(f"Robot thread: Exception in runrobot: {e}")
        traceback.print_exc()
    finally:
        print("Robot thread: Loop finished. Cleaning up in main.")
        # Hardware shutdown is handled by the main() function's finally block


def main():
    parser = argparse.ArgumentParser(description="Multithreaded Robot Control")
    parser.add_argument("--display", action="store_true", help="Display map in real-time using TkAgg backend")
    parser.add_argument("--x", type=float, default=0.0, help="Initial x-coordinate (default: 0.0)")
    parser.add_argument("--y", type=float, default=0.0, help="Initial y-coordinate (default: 0.0)")
    parser.add_argument("--heading", type=int, default=0, choices=range(8), help="Initial heading direction (0-7, default: 0)")
    args = parser.parse_args()

    if args.display:
        os.environ["display_map"] = "on"
        print("Display map enabled - using TkAgg backend")
    else:
        if "display_map" in os.environ:
            del os.environ["display_map"] # Ensure it's off if not specified

    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon. Is it running?")
        return

    print("Hardware: Initializing...")
    robot_hardware = Robot(io) 
    # Note: DriveSystem and LineSensor are part of robot_hardware
    adc = ADC(io) 
    behaviors_obj = Behaviors(robot_hardware.drive_system, robot_hardware.sensors, adc)
    print("Hardware: Initialization complete.")

    initial_x = args.x
    initial_y = args.y
    initial_heading = args.heading

    shared_data = SharedData()
    shared_data.pose = (initial_x, initial_y, initial_heading) # Initialize shared pose

    ui_thread = threading.Thread(name="UIThread", target=runui, args=(shared_data,))
    ui_thread.daemon = True 
    ui_thread.start()
    print("Main thread: UI worker thread started.")

    try:
        runrobot(shared_data, behaviors_obj, robot_hardware, initial_x, initial_y, initial_heading)
    except Exception as e:
        print(f"Main thread: Uncaught exception from runrobot call: {e}")
        traceback.print_exc()
    finally:
        print("Main thread: runrobot has exited. Initiating shutdown...")
        # Ensure UI thread knows to quit if it hasn't already
        if shared_data.acquire():
            if shared_data.mode != -1: # If not already quitting
                 print("Main thread: Signaling UI thread to quit.")
                 shared_data.mode = -1 
            shared_data.release()
        
        # Wait briefly for UI thread to process quit command (optional, as it's a daemon)
        # ui_thread.join(timeout=1.0) 
        # print("Main thread: UI thread join attempt complete.")

        print("Main thread: Shutting down hardware...")
        if hasattr(robot_hardware, 'stop') and callable(getattr(robot_hardware, 'stop')):
             robot_hardware.stop() # This should also call io.stop()
        else: # Fallback
            print("Main thread: robot_hardware.stop() not found or not callable. Attempting manual stop.")
            if hasattr(robot_hardware, 'drive_system') and hasattr(robot_hardware.drive_system, 'stop'):
                robot_hardware.drive_system.stop()
            if hasattr(io, "connected") and io.connected:
                io.stop()
        print("Main thread: Hardware shutdown complete.")
        print("Main thread: Exiting program.")

if __name__ == "__main__":
    main()
