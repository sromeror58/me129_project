import pigpio
import traceback
import os
import argparse
import copy 
from drive_system import DriveSystem
from sensor import LineSensor
from behaviors import Behaviors
from pose import Pose
from magnetometer import ADC
from proximitysensor import ProximitySensor
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

def initialization_helper(behaviors, robot, heading, x, y):
    """
    Initializes the intersection at (x, y) with street status based on sensor data.

    Uses line-following behavior to check for a U-turn and detect a road ahead, then sets
    the appropriate street status in the current and opposite headings.

    Args:
        behaviors (Behaviors): Behavior object
        robot (Robot): Robot object
        heading (int): Current heading (0–7).
        x (int): X-coordinate of the location.
        y (int): Y-coordinate of the location.

    Returns:
        tuple: A dict mapping (x, y) to an Intersection, and the updated heading.
    """
    isUturn, travel_time, road_ahead = behaviors.line_follow()
    current_streets = [STATUS.UNKNOWN] * 8
    if isUturn:
        # checking if we hit a u-turn meaning we have a deadend in that heading
        # then we reach a intersection where we will make the intersection at
        current_streets[heading] = STATUS.DEADEND
        heading = (heading + 4) % 8
        # setting the street ahead of the intersection unexplored if the we have road_ahead
        # else we keep nonexistent (accounting the pull forward detector)
        current_streets[heading] = (
            STATUS.UNEXPLORED if road_ahead else STATUS.NONEXISTENT
        )
    else:
        # the case when we reach an intersection without a u-turn
        current_streets[heading] = (
            STATUS.UNEXPLORED if road_ahead else STATUS.NONEXISTENT
        )
    initial_intersection = Intersection(x, y, current_streets)
    return {(x, y): initial_intersection}, heading

def runrobot(shared: SharedData, behaviors: Behaviors, robot_hardware: Robot, initial_x: float, initial_y: float, initial_heading: int):
    """
    Main robot control loop, running in the main thread.
    It interacts with the UI thread via the shared SharedData object.
    """
    print(f"Robot thread: Initializing with pose ({initial_x}, {initial_y}, {initial_heading})")
    current_pose = Pose(initial_x, initial_y, initial_heading)
    
    print("Robot thread: Initializing first intersection...")
    initial_intersections, updated_heading = initialization_helper(behaviors, robot_hardware, initial_heading, initial_x, initial_y)
    current_pose.heading = updated_heading  
    
    map_obj = Map(current_pose, initial_intersections)
    # Initialize shared.map after map_obj is created
    if shared.acquire():
        try:
            shared.map = copy.deepcopy(map_obj)
        finally:
            shared.release()
    map_obj.plot(current_pose)

    # Local to robot thread
    num_streets_to_goal = [0, []] 
    running = True
    
    # To store the mode before pausing, to allow resume to correct mode
    previous_autonomous_mode_before_pause = 1 
    has_printed_pause_message = False 

    try:
        while running:
            local_mode = 0
            local_goal = None
            local_command = None
            local_map_filename = None
            local_ui_pose_update = None
            local_step_mode = False
            local_map = None 
            
            if shared.acquire():
                try:
                    local_mode = shared.mode
                    local_goal = shared.goal
                    local_command = shared.command
                    if local_command: # If there is a command, consume it
                        shared.command = None
                    
                    local_map_filename = shared.map_filename
                    # map_filename is consumed after use by specific commands

                    local_ui_pose_update = shared.pose 
                    
                    local_step_mode = shared.step_mode
                    # step_mode is NOT consumed here by robot thread; UI thread resets it or robot thread consumes it when stepping

                    # Update shared.pose with the robot's current position for UI
                    shared.pose = (current_pose.x, current_pose.y, current_pose.heading)
                    
                    # Make a deep copy of the map for this iteration's logic
                    # Ensure map_obj (robot's working copy) is updated if shared.map was changed by UI (e.g. load)
                    if shared.map is not None:
                        if map_obj is None or id(map_obj) != id(shared.map): # Check if shared.map is a new instance
                            pass # map_obj will be updated after UI commands section if needed
                        local_map = copy.deepcopy(shared.map)
                    else: 
                        if map_obj: # if robot has a map_obj
                             if shared.acquire(): # Update shared map with robot's current map_obj
                                 try:
                                     shared.map = copy.deepcopy(map_obj)
                                 finally: shared.release()
                             local_map = copy.deepcopy(map_obj)

                finally:
                    shared.release()

            if local_mode == -1:
                print("Robot thread: Quit command received from UI.")
                running = False
                continue

            # Process High-Priority UI Commands (state changes, no immediate movement)
            if local_command == 'load':
                if local_map_filename:
                    try:
                        print(f"Robot thread: Loading map from {local_map_filename}.pickle")
                        new_map = Map.load_map(local_map_filename)
                        print(f"Map '{local_map_filename}' loaded. Use 'pose' command to set robot's position on this map.")
                        map_obj = new_map  # Robot's working map is now the new_map
                        if shared.acquire(): # Update shared.map for UI
                            try:
                                shared.map = copy.deepcopy(new_map)
                            finally:
                                shared.release()
                        local_map = copy.deepcopy(new_map) # Update local_map copy for consistency
                        map_obj.plot(current_pose) 
                    except FileNotFoundError:
                        print(f"Robot thread: Error loading map. File '{local_map_filename}.pickle' not found.")
                    except Exception as e:
                        print(f"Robot thread: Error loading map '{local_map_filename}': {e}")
                    if shared.acquire(): 
                        shared.map_filename = None
                        shared.release()
                local_command = None 

            elif local_command == 'save':
                if local_map_filename:
                    print(f"Robot thread: Saving map to {local_map_filename}.pickle")
                    map_obj.save_map(filename=local_map_filename) # Save from robot's working map_obj
                    if shared.acquire(): 
                        shared.map_filename = None
                        shared.release()
                local_command = None

            elif local_command == 'pose':
                if local_ui_pose_update: 
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
            
            # --- Determine potential_action_cmd (robot's next potential physical action) ---
            potential_action_cmd = None
            
            # Update previous_autonomous_mode_before_pause if currently in an autonomous mode
            if local_mode == 1: # Explore
                previous_autonomous_mode_before_pause = 1
            elif local_mode == 2: # Goal
                previous_autonomous_mode_before_pause = 2

            # Determine the mode to use for calculating the potential action
            mode_to_calculate_action_for = local_mode
            if local_mode == 3 and local_step_mode: # If paused and stepping
                mode_to_calculate_action_for = previous_autonomous_mode_before_pause
            
            current_intersection = map_obj.getintersection(current_pose.x, current_pose.y)
            if not current_intersection and mode_to_calculate_action_for in [1,2]: # Only critical if an autonomous action is expected
                print(f"CRITICAL ERROR: Robot at ({current_pose.x},{current_pose.y}) which is not a known intersection! Check map or use 'pose'. Switching to manual.")
                if shared.acquire():
                    shared.mode = 0
                    shared.release()
                local_mode = 0 # Effectively switch to manual for this cycle
                mode_to_calculate_action_for = 0 # Don't attempt autonomous calculation

            if local_mode == 0: # Manual action from UI
                if local_command in ['left', 'right', 'straight']:
                    potential_action_cmd = local_command[0]
                    print(f"Robot thread: Manual action identified: {potential_action_cmd}")

            elif mode_to_calculate_action_for == 1: # Explore Logic
                unexplored_streets_at_current = map_obj.get_unexplored_streets(current_pose.x, current_pose.y)
                target_heading_for_explore = None
                if unexplored_streets_at_current:
                    closest_street_info = min(unexplored_streets_at_current, key=lambda x_info: min((x_info[0] - current_pose.heading) % 8, (current_pose.heading - x_info[0]) % 8))
                    target_heading_for_explore = closest_street_info[0]
                    print(f"Robot thread: Exploring - Target heading at current intersection: {target_heading_for_explore}")
                    # Accurate turn optimization for explore
                    num_streets_to_goal = [0, []] # Reset for explore's simple turns or calc new accurate turn
                    if target_heading_for_explore != current_pose.heading:
                        heading_diff = (target_heading_for_explore - current_pose.heading) % 8
                        turn_dir = 1 if heading_diff <= 4 else -1
                        temp_h = current_pose.heading
                        streets_list = []
                        # current_intersection should be valid here if this block is reached
                        current_int_for_turn_calc = map_obj.getintersection(current_pose.x, current_pose.y) # Re-fetch for safety
                        can_do_accurate_turn = True
                        if current_int_for_turn_calc:
                            while temp_h != target_heading_for_explore:
                                temp_h = (temp_h + turn_dir) % 8
                                if current_int_for_turn_calc.streets[temp_h] == STATUS.UNKNOWN:
                                    can_do_accurate_turn = False; break
                                if current_int_for_turn_calc.streets[temp_h] != STATUS.NONEXISTENT:
                                    streets_list.append(temp_h)
                            if can_do_accurate_turn and streets_list: # Ensure target street is part of list
                                num_streets_to_goal = [turn_dir, streets_list]
                            else: # Default to simple turn, clear list
                                num_streets_to_goal = [0,[]] 
                        else: # Should not happen if initial check passed
                            num_streets_to_goal = [0,[]]


                else: # No unexplored streets at current intersection
                    nearest_unexplored_coord = map_obj.find_nearest_unexplored(current_pose.x, current_pose.y)
                    if nearest_unexplored_coord is None:
                        print("Robot thread: Map fully explored! Switching to manual mode.")
                        if shared.acquire(): shared.mode = 0; shared.release()
                        local_mode = 0 # Update local_mode to prevent action_cmd by pause/step logic later
                        # potential_action_cmd remains None
                    else:
                        print(f"Robot thread: Exploring - Pathing to nearest unexplored intersection: {nearest_unexplored_coord}")
                        map_obj.setstreet(nearest_unexplored_coord[0], nearest_unexplored_coord[1])
                        current_intersection_with_path = map_obj.getintersection(current_pose.x, current_pose.y) 
                        if current_intersection_with_path and current_intersection_with_path.direction is not None:
                            target_heading_for_explore = current_intersection_with_path.direction
                        else:
                            print(f"Robot thread: Exploring - No path to {nearest_unexplored_coord}. Stuck? Switching to manual.")
                            if shared.acquire(): shared.mode = 0; shared.release()
                            local_mode = 0
                
                if target_heading_for_explore is not None and (local_mode == 1 or (local_mode == 3 and local_step_mode and previous_autonomous_mode_before_pause == 1)):
                    if current_pose.heading == target_heading_for_explore:
                        potential_action_cmd = 's'
                    else:
                        # If num_streets_to_goal is populated, it implies an accurate turn.
                        # Otherwise, simple diff.
                        if num_streets_to_goal[1]: # Using accurate turn path
                             potential_action_cmd = 'l' if num_streets_to_goal[0] > 0 else 'r'
                        else: # Simple turn decision
                            heading_diff = (target_heading_for_explore - current_pose.heading) % 8
                            potential_action_cmd = 'l' if heading_diff <= 4 else 'r'
                print(f"Robot thread: Explore action identified: {potential_action_cmd}, num_streets_to_goal: {num_streets_to_goal}")


            elif mode_to_calculate_action_for == 2: # Goal Logic
                if local_goal is None:
                    print("Robot thread: Goal mode, but no goal set. Switching to manual.")
                    if shared.acquire(): shared.mode = 0; shared.release()
                    local_mode = 0
                elif (current_pose.x, current_pose.y) == local_goal:
                    print("Robot thread: Goal reached! Switching to manual mode.")
                    if shared.acquire(): shared.mode = 0; shared.goal = None; shared.release()
                    local_mode = 0
                    map_obj.setstreet(None, None) 
                    num_streets_to_goal = [0, []]
                    map_obj.plot(current_pose) 
                elif current_intersection: 
                    map_obj.setstreet(local_goal[0], local_goal[1])
                    current_intersection_with_path = map_obj.getintersection(current_pose.x, current_pose.y)
                    if current_intersection_with_path and current_intersection_with_path.direction is not None:
                        if current_pose.heading == current_intersection_with_path.direction:
                            potential_action_cmd = 's'
                            print(f"Robot thread: Goal Mode - Going straight towards {local_goal}")
                        else: 
                            if not num_streets_to_goal[1] or num_streets_to_goal[0] == 0 : # Calculate turn path if not already doing so or completed previous
                                heading_diff = (current_intersection_with_path.direction - current_pose.heading) % 8
                                num_streets_to_goal[0] = 1 if heading_diff <= 4 else -1 
                                temp_h = current_pose.heading
                                streets_list = []
                                while temp_h != current_intersection_with_path.direction:
                                    temp_h = (temp_h + num_streets_to_goal[0]) % 8
                                    if current_intersection_with_path.streets[temp_h] != STATUS.NONEXISTENT:
                                        streets_list.append(temp_h)
                                num_streets_to_goal[1] = streets_list
                                print(f"Robot thread: Goal Mode - Calculated turn path: {num_streets_to_goal}")
                            potential_action_cmd = 'l' if num_streets_to_goal[0] > 0 else 'r'
                            print(f"Robot thread: Goal Mode - Turning {potential_action_cmd} towards {current_intersection_with_path.direction}. Path: {num_streets_to_goal[1]}")
                    else:
                        print(f"Robot thread: Goal Mode - No valid path to goal {local_goal}. Switching to manual.")
                        if shared.acquire(): shared.mode = 0; shared.goal = None; shared.release()
                        local_mode = 0
                        map_obj.setstreet(None, None)
                        num_streets_to_goal = [0, []]
                print(f"Robot thread: Goal action identified: {potential_action_cmd}")


            # --- Determine final action_cmd based on actual local_mode (paused or not) ---
            action_cmd = None 
            if local_mode == 3: # Paused
                if not has_printed_pause_message:
                    print("Robot thread: Paused.")
                    has_printed_pause_message = True
                
                if local_step_mode:
                    print("Robot thread: Step command active.")
                    action_cmd = potential_action_cmd # Use the action calculated based on previous_autonomous_mode
                    if shared.acquire():
                        shared.step_mode = False # Consume step
                        shared.release()
                    if action_cmd:
                        print(f"Robot thread: Stepping with action: {action_cmd}")
                    else:
                        print("Robot thread: Step requested, but no autonomous action determined (e.g. goal reached or explore stuck).")
                # else: action_cmd remains None (paused, not stepping, so robot does nothing)
            else: # Not paused (local_mode is 0, 1, or 2)
                has_printed_pause_message = False # Reset pause message flag
                action_cmd = potential_action_cmd # Use the action determined by manual, explore, or goal logic

            
            # --- Execute Physical Action ---
            if action_cmd:
                print(f"Robot thread: Executing physical action: {action_cmd}")
                pose0 = current_pose.clone()
                
                # Pre-check for straight action against NONEXISTENT street
                if action_cmd == 's':
                    # current_intersection might have been updated by setstreet in goal/explore path logic
                    current_int_for_straight = map_obj.getintersection(current_pose.x, current_pose.y)
                    if current_int_for_straight and current_int_for_straight.streets[current_pose.heading] == STATUS.NONEXISTENT:
                        print(f"Robot thread: Cannot go straight from ({current_pose.x},{current_pose.y}) heading {current_pose.heading}, street is NONEXISTENT.")
                        action_cmd = None # Cancel action

                if action_cmd == 'l':
                    turnAngle, _ = behaviors.turn_to_next_street("left")
                    if not num_streets_to_goal[1]: # Simple turn or exploration's non-accurate turn
                        current_pose.calcturn(turnAngle, True)
                        # Heading correction for simple/exploration turns
                        intersection_after_turn = map_obj.getintersection(current_pose.x, current_pose.y)
                        if intersection_after_turn and intersection_after_turn.streets[current_pose.heading] == STATUS.NONEXISTENT:
                            turned_angle_abs = abs(turnAngle)
                            dh = (current_pose.heading - pose0.heading) % 8
                            da_lower = (dh - 1) * 45
                            da_upper = (dh + 1) * 45
                            if abs(turned_angle_abs - da_lower) < abs(turned_angle_abs - da_upper):
                                street_check = intersection_after_turn.streets[(current_pose.heading - 1) % 8]
                                if street_check in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.UNEXPLORED]:
                                    current_pose.heading = (current_pose.heading - 1) % 8
                                    print("Robot thread: Adjusted heading left due to NONEXISTENT street after simple turn.")
                            else:
                                street_check = intersection_after_turn.streets[(current_pose.heading + 1) % 8]
                                if street_check in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.UNEXPLORED]:
                                    current_pose.heading = (current_pose.heading + 1) % 8
                                    print("Robot thread: Adjusted heading right due to NONEXISTENT street after simple turn.")
                    else: # Accurate turn leg for goal mode or explore mode's accurate turn
                        current_pose.heading = num_streets_to_goal[1].pop(0)
                        if not num_streets_to_goal[1]: num_streets_to_goal[0] = 0 
                    current_street_blocked = behaviors.check_blockage()
                    if current_street_blocked:
                        map_obj.check_and_set_blocked(current_pose, current_street_blocked)
                    map_obj.outcomeA(pose0, current_pose, True)

                elif action_cmd == 'r':
                    turnAngle, _ = behaviors.turn_to_next_street("right")
                    if not num_streets_to_goal[1]:
                        current_pose.calcturn(turnAngle, False)
                        # Heading correction for simple/exploration turns
                        intersection_after_turn = map_obj.getintersection(current_pose.x, current_pose.y)
                        if intersection_after_turn and intersection_after_turn.streets[current_pose.heading] == STATUS.NONEXISTENT:
                            turned_angle_abs = abs(turnAngle)
                            dh = (pose0.heading - current_pose.heading) % 8 # Note: dh calculation is different for right turns
                            da_lower = (dh + 1) * 45 # Adjusted for right turn logic from old_main
                            da_upper = (dh - 1) * 45 # Adjusted
                            if abs(turned_angle_abs - da_lower) < abs(turned_angle_abs - da_upper): # Check logic for da_lower/da_upper
                                street_check = intersection_after_turn.streets[(current_pose.heading + 1) % 8]
                                if street_check in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.UNEXPLORED]:
                                    current_pose.heading = (current_pose.heading + 1) % 8
                                    print("Robot thread: Adjusted heading right due to NONEXISTENT street after simple turn.")
                            else:
                                street_check = intersection_after_turn.streets[(current_pose.heading - 1) % 8]
                                if street_check in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.UNEXPLORED]:
                                    current_pose.heading = (current_pose.heading - 1) % 8
                                    print("Robot thread: Adjusted heading left due to NONEXISTENT street after simple turn.")
                    else:
                        current_pose.heading = num_streets_to_goal[1].pop(0)
                        if not num_streets_to_goal[1]: num_streets_to_goal[0] = 0
                    current_street_blocked = behaviors.check_blockage()
                    if current_street_blocked:
                        map_obj.check_and_set_blocked(current_pose, current_street_blocked)
                    map_obj.outcomeA(pose0, current_pose, False)

                elif action_cmd == 's':
                    isUturn, _, road_ahead = behaviors.line_follow()
                    if not isUturn:
                        current_pose.calcmove()
                        current_street_blocked = behaviors.check_blockage()
                        if current_street_blocked:
                            map_obj.check_and_set_blocked(current_pose, current_street_blocked)
                        map_obj.outcomeB(pose0, current_pose, road_ahead)
                    else:
                        current_pose.calcuturn()
                        current_street_blocked = behaviors.check_blockage()
                        if current_street_blocked:
                            map_obj.check_and_set_blocked(current_pose, current_street_blocked)
                        map_obj.outcomeC(pose0, current_pose, road_ahead)
                
                if action_cmd: # If an action was actually performed

                    # Update shared map after successful action
                    if shared.acquire():
                        try:
                            shared.map = copy.deepcopy(map_obj)
                            shared.map.plot(current_pose)
                        finally:
                            shared.release()
            
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

    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon. Is it running?")
        return

    print("Hardware: Initializing...")

    robot_hardware = Robot(io) 
    # Note: DriveSystem and LineSensor are part of robot_hardware
    adc = ADC(io) 
    proximity = ProximitySensor(io)
    behaviors_obj = Behaviors(robot_hardware.drive_system, robot_hardware.sensors, adc, proximity)
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
        
        print("Main thread: Shutting down hardware...")
        if hasattr(robot_hardware, 'stop') and callable(getattr(robot_hardware, 'stop')):
            proximity.shutdown()
            robot_hardware.stop() # This should also call io.stop()
        else: # Fallback
            print("Main thread: robot_hardware.stop() not found or not callable. Attempting manual stop.")
            if hasattr(robot_hardware, 'drive_system') and hasattr(robot_hardware.drive_system, 'stop'):
                proximity.shutdown()
                robot_hardware.drive_system.stop()
            if hasattr(io, "connected") and io.connected:
                io.stop()
        print("Main thread: Hardware shutdown complete.")
        print("Main thread: Exiting program.")

if __name__ == "__main__":
    main()
