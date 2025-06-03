# Imports
import pigpio
import traceback
import os
import argparse
import copy
from drive_system import DriveSystem, turn_fit
from sensor import LineSensor
from behaviors import Behaviors
from pose import Pose
from magnetometer import ADC
from map import Map, Intersection, STATUS
import time
from proximitysensor import ProximitySensor
from ui_main import SharedData
import threading
from ros import runros

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
    original_heading = heading
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
    # initializing non-existent streets to opposite heading +- 45
    opposite_heading = (original_heading + 4) % 8
    current_streets[(opposite_heading + 1) % 8] = STATUS.NONEXISTENT
    current_streets[(opposite_heading - 1) % 8] = STATUS.NONEXISTENT
    
    # if road ahead, block out +- 45 headings
    if road_ahead:
        current_streets[(heading + 1) % 8] = STATUS.NONEXISTENT
        current_streets[(heading - 1) % 8] = STATUS.NONEXISTENT
    initial_intersection = Intersection(x, y, current_streets)
    return {(x, y): initial_intersection}, heading


def initialize_map(behaviors, robot, heading, x, y, ask=False, no_ask_filename=""):
    """
    Asks the user to load a map or create a new one, then returns the map and starting pose.
    """
    choice = "n"
    if ask:
        choice = input("Load existing map? (y/n): ").strip().lower()
    if choice == "y":
        # only querying file name without extension (i.e. without .pickle at the end for simplicity)
        filename = input("Enter filename (default: mymap.pickle): ").strip() or "mymap"
        print(f'filename inputted: {filename}')
        map_obj = Map.load_map(filename=filename)
        x = int(input("Enter starting x-coordinate: "))
        y = int(input("Enter starting y-coordinate: "))
        heading = int(input("Enter starting heading (0–7): "))
        intersection_dictionary, heading = initialization_helper(
            behaviors, robot, heading, x, y
        )
        pose = Pose(x, y, heading)
        return map_obj, pose
    else:
        intersection_dictionary, heading = initialization_helper(
            behaviors, robot, heading, x, y
        )
        pose = Pose(x, y, heading)
        if no_ask_filename:
            map_obj = Map.load_map(no_ask_filename)
        else:
            map_obj = Map(pose, intersection_dictionary)
        return map_obj, pose


def choose_best_angle(time_estimate, mag_estimate, angles, pose):
    weight_time = 0.3
    weight_mag = 0.7

    # Convert 360 degree angles to 0 degree
    # change to this and fix if we have turns > 360
    # angles = [0 if abs(angle) >= 360 else angle for angle in angles]
    # angles = [0 if abs(angle) == 360 else angle for angle in angles]

    # Adjust weights if it's detected as a near full turn
    if abs(mag_estimate) <= 25 and abs(time_estimate) >= 120:
        print("BAM 360 turn")
        weight_time = 0
        weight_mag = 1.0
    elif abs(mag_estimate) <= 120 or abs(time_estimate) <= 120:
        weight_time = 0.4
        weight_mag = 0.6
    
    weighted_average = weight_time * time_estimate + weight_mag * mag_estimate
    print(f'Weighted average: {weighted_average}')
    best_angle = min(angles, key=lambda angle: abs(weighted_average - angle))
    print(f'Chosen best angle: {best_angle}')
    return best_angle, (pose.heading + best_angle//45) % 8

def simple_brain(behaviors, robot, shared, x=0.0, y=0.0, heading=0):
    """
    Simple brain function that handles robot navigation and mapping.

    This function implements a basic control loop that:
    1. Accepts user commands for robot movement
    2. Updates the robot's pose based on the commands
    3. Updates the map based on the robot's movement outcomes
    4. Visualizes the map and robot's current state

    Args:
        behaviors (Behaviors): Robot behaviors instance for executing movements
        robot (Robot): Robot instance for hardware control
        x (float): Initial x-coordinate x(default: 0.0)
        y (float): Initial y-coordinate (default: 0.0)
        heading (int): Initial heading direction (0-7, default: 0)
    """
    map, pose = initialize_map(behaviors, robot, heading, x, y, ask=True)
    map.plot(pose)

    goal = None  # Track current goal coordinates
    num_streets_to_goal = [0, []] # Track turns needed to get to goal

    shared.mode = 0

    while True:

        # Acquire shared data
        if shared.acquire():
            try:
                mode = shared.mode
                take_step = shared.take_step
                goal = shared.goal 
                command = shared.command 
                map_name = shared.map_filename
                before_pause = shared.before_pause

            finally:
                shared.release()

        # If escaped goal following / explore, reset num_streets_to_goal
        if mode in [0, 3]:
            num_streets_to_goal = [0, []]

        # GO THROUGH COMMANDS
        if command == 'save':
            print("Saving plot...")
            map.save_map(filename=map_name)
            if shared.acquire():
                try:
                    shared.command = None
                finally:
                    shared.release()
                    continue

        elif command == 'clear':
            print("Clearing blocked streets...")
            if shared.acquire():
                try:
                    map.clear_blocked()
                    map.plot(pose)
                    shared.command = None
                finally:
                    shared.release()
                    continue

        elif command == 'pose':
            print("Setting pose...")
            if shared.acquire():
                try:
                    pose_tuple = shared.pose
                    pose = Pose(pose_tuple[0], pose_tuple[1], pose_tuple[2])
                    map.plot(pose)
                    shared.command = None
                finally:
                    shared.release()
                    continue
        elif command == 'show':
            print("Updating map")
            map.plot(pose)
            if shared.acquire():
                try:
                    shared.command = None
                finally:
                    shared.release()
                    continue
        elif command == 'load':
            print("Loading map...")
            map, pose = initialize_map(behaviors, robot, heading, 0.0, 0.0, False, map_name)
            map.plot(pose)
            if shared.acquire():
                try:
                    shared.command = None
                finally:
                    shared.release()
                    continue

        # GO THROUGH MODES
        if mode == -1:
            print("Quitting...")
            robot.stop()
            map.close()
            break

        if mode == 1:
            all_streets_known = True
            for x, y in map.intersections:
                intersection = map.getintersection(x, y)
                for heading in range(8):
                    # Check if street is unknown or unblocked unexplored
                    if (intersection.streets[heading] == STATUS.UNKNOWN or 
                        (intersection.streets[heading] == STATUS.UNEXPLORED and not intersection.blocked[heading])):
                        all_streets_known = False
            if all_streets_known:
                print("Map fully explored! Returning to manual mode.")
                if shared.acquire():
                    try:
                        shared.mode = 0
                        shared.goal = None
                    finally:
                        shared.release()
                continue
    
        # If not manual-mode: That is, goal-following, autonomous, step mode
        if mode != 0 and not (mode == 3 and not take_step):
            try:
                taking_step = take_step
                # If we have a goal, check if we've reached it
                if goal is not None:

                    if take_step:
                        if shared.acquire():
                            try:
                                shared.take_step = False
                            finally:
                                shared.release()

                    if (pose.x, pose.y) == goal:

                        # If was in goal mode, go to manual
                        # If was in autonomous mode, clear goal
                        if shared.acquire():
                            try:
                                if shared.mode == 2:
                                    shared.mode = 0
                                    print("Goal reached! Returning to manual mode from goal mode")
                                elif shared.mode == 1:
                                    print("Goal reached! Continuing autonomous mode")
                                elif shared.mode == 3 and before_pause == 2:
                                    print("Goal reached after the final step! Returning to manual mode from goal mode")
                                    shared.mode = 0
                                shared.goal = None
                            finally:
                                shared.release()
                        map.setstreet(None, None)  # Clear optimal path tree
                        map.plot(pose)
                        continue

                    map.setstreet(goal[0], goal[1])
                    # We haven't reached goal, so get current intersection and check optimal direction
                    current = map.getintersection(pose.x, pose.y)
                    if current.direction is not None:

                        # If current heading doesn't match optimal direction, need to turn
                        if pose.heading != current.direction:

                            # If we haven't already, calculate number of turns needed
                            if not num_streets_to_goal[1]:

                                current_heading = pose.heading
                                streets_encountered = 0

                                # Calculate the difference in heading turning left
                                heading_diff = (current.direction - current_heading) % 8

                                if heading_diff <= 4:
                                    num_streets_to_goal[0] = 1 # Left turns
                                else:
                                    num_streets_to_goal[0] = -1 # Right turns

                                # Get list of streets encountered when turning in intended direction
                                while current_heading != current.direction:
                                    current_heading = (current_heading + num_streets_to_goal[0]) % 8
                                    if current.streets[current_heading] != STATUS.NONEXISTENT: # Assuming streets have been mapped
                                        num_streets_to_goal[1].append(current_heading)
                                
                            if num_streets_to_goal[0] > 0:
                                command = 'left'  # Turn left
                            else:
                                command = 'right'  # Turn right

                        else:
                            print("Going straight along optimal path")
                            command = 'straight'
                    else:
                        if taking_step:
                            print("No valid path to goal. No step taken: clearing goal and staying in paused.")
                            if shared.acquire():
                                try:
                                    shared.goal = None
                                    shared.mode = 0
                                finally:
                                    shared.release()
                        else:
                            print("No valid path to goal. Returning to manual mode.")
                            if shared.acquire():
                                try:
                                    shared.goal = None
                                    shared.mode = 0
                                finally:
                                    shared.release()
                        map.setstreet(None, None)  # Clear optimal path tree
                        map.plot(pose)
                        continue

                else:
                    if mode == 1 or take_step:

                        if take_step:
                            if shared.acquire():
                                try:
                                    shared.take_step = False
                                finally:
                                    shared.release()
                        # In the process of making an accurate turn
                        if num_streets_to_goal[1]:
                            if num_streets_to_goal[0] > 0:
                                command = "left"  # Turn left
                            else:
                                command = "right"  # Turn right

                        else:
                            # Get unexplored streets and find the one closest to current heading
                            unexplored_streets = map.get_unexplored_streets(pose.x, pose.y)
                            closest_heading = min(unexplored_streets, key=lambda x: min((x[0] - pose.heading) % 8, (pose.heading - x[0]) % 8))[0] if unexplored_streets else None
                            print(unexplored_streets)
                            print(closest_heading)

                            if closest_heading is not None:
                                if closest_heading == pose.heading:
                                    # Before going straight, check if the street is blocked
                                    if map.getintersection(pose.x, pose.y).blocked[closest_heading]:
                                        print("Cannot go straight: street is blocked!")
                                        # Mark the street as blocked and continue exploration
                                        map.getintersection(pose.x, pose.y).updateStreet(closest_heading, STATUS.BLOCKED)
                                        # Recalculate path to nearest unexplored intersection
                                        nearest = map.find_nearest_unexplored(pose.x, pose.y)
                                        if nearest is not None:
                                            goal = nearest
                                            map.setstreet(goal[0], goal[1])
                                        continue
                                    command = "straight"

                                else:
                                    # Local exploration - choose first unexplored street
                                    heading_diff = (closest_heading - pose.heading) % 8

                                    # Can we make an accurate turn?
                                    # If all the streets are known between the current heading and target heading, then we can make the turn more accurate
                                    # So (1) all the streets between current and target heading are NOT UNKNOWN, (2) target is UNEXPLORED and NOT UNKNOWN
                                    current_heading = pose.heading
                                    if heading_diff <= 4:
                                        num_streets_to_goal[0] = 1 # Left turns
                                    else:
                                        num_streets_to_goal[0] = -1 # Right turns
                                    # Check all streets between current heading and target heading
                                    current = map.getintersection(pose.x, pose.y)
                                    while current_heading != closest_heading:
                                        current_heading = (current_heading + num_streets_to_goal[0]) % 8
                                        # If any street is UNKNOWN, we can't make an accurate turn
                                        if current.streets[current_heading] == STATUS.UNKNOWN:
                                            num_streets_to_goal[1] = [] # Clear the streets to encounter
                                            break
                                        # Otherwise, if the street EXISTS, then we should take note of it
                                        elif current.streets[current_heading] != STATUS.NONEXISTENT:
                                            num_streets_to_goal[1].append(current_heading)                    
                                    # Also verify target street is UNEXPLORED
                                    if current.streets[closest_heading] != STATUS.UNEXPLORED:
                                        num_streets_to_goal[1] = []

                                    # Now we can make an accurate turn, if it exists; otherwise, a less accurate turn
                                    if heading_diff <= 4:
                                        command = "left" 
                                    else:
                                        command = "right" 

                            else:
                                # No unexplored streets here, find nearest unexplored intersection
                                nearest = map.find_nearest_unexplored(pose.x, pose.y)
                                
                                if nearest is None:
                                    print("Map fully explored! Returning to manual mode. uhuhuhu")
                                    if shared.acquire():
                                        try:
                                            shared.mode = 0
                                            shared.goal = None
                                        finally:
                                            shared.release()
                                else:
                                    goal = nearest
                                    if shared.acquire():
                                        try:
                                            shared.goal = copy.deepcopy(nearest)
                                        finally:
                                            shared.release()
                                    map.setstreet(goal[0], goal[1])

                                map.plot(pose)
                                if shared.goal and shared.mode != 0:
                                    print(f"Setting goal to unexplored intersection at {goal}")
                                # continue  # Let the goal-directed navigation handle the movement

            except KeyboardInterrupt:
                robot.stop()
                map.close()
                break


        ##############################################################
        # LEFT, RIGHT, STRAIGHT RESULTING FROM EITHER MANUAL OR GOAL #
        ##############################################################

        if command == 'left':
            # If paused and instructed to take a step
            # perform left turn
            print("Turning left...")
            # Taking the possible turn angles
            # possible_turns = map.get_unexplored_streets(pose.x, pose.y)
            # possible_headings = [possible_turns[i][0] for i in range(len(possible_turns))]
            possible_angles = map.get_possible_angles(pose, turn_direction="left")
            possible_headings = [(pose.heading + angle//45) % 8 for _, angle in possible_angles ]
            # Store current pose values before turning
            pose0 = pose.clone()
            # returning the turn angle and turn time
            turnAngle, turn_time = behaviors.turn_to_next_street("left")
            time_turn_estimate = turn_fit(turn_time)
            # print(f'time turn estimate: {time_turn_estimate} degrees')
            print(f'Turn time: {turn_time}\t time-based turn:{time_turn_estimate}\t mag-based turn:{turnAngle}\t list of angles:{possible_headings}')
            chosen_angle, chosen_offset = choose_best_angle(time_turn_estimate, turnAngle, [angle for _, angle in possible_angles], pose)
            if not num_streets_to_goal[1]: # If turning without goal-following
                pose.calcturn(chosen_angle, True)
                # pose.calcturn(turnAngle, True)
            else: # If turning without goal-following
                pose.heading = num_streets_to_goal[1].pop(0)
            intersection = map.getintersection(pose.x, pose.y)
            # turned_angle = abs(turnAngle)
            turned_angle = abs(chosen_angle)
            dh = (pose.heading - pose0.heading) % 8
            da_lower = (dh - 1) * 45
            da_upper = (dh + 1) * 45
            if intersection.streets[pose.heading] == STATUS.NONEXISTENT:
                if abs(turned_angle - da_lower) < abs(turned_angle - da_upper):
                    street = intersection.streets[(pose.heading - 1) % 8] 
                    # Only adjust heading if the next closest street is already explored (CONNECTED or DEADEND)
                    if street in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.UNEXPLORED]:
                        pose.heading = (pose.heading - 1) % 8
                        print("BINGAAAAA")
                else:
                    street = intersection.streets[(pose.heading + 1) % 8] 
                    if street in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.UNEXPLORED]:
                        pose.heading = (pose.heading + 1) % 8
                        print("BINGBBBBB")
            map.outcomeA(pose0, pose, True)
            time.sleep(0.2)
            behaviors.realign()
            if shared.acquire():
                try:
                    shared.command = None
                finally:
                    shared.release()

        elif command == 'right':
            # perform right turn
            print("Turning right...")
            # Taking the possible turn angles
            # possible_turns = map.get_unexplored_streets(pose.x, pose.y)
            # possible_headings = [possible_turns[i][0] for i in range(len(possible_turns))]
            possible_angles = map.get_possible_angles(pose, turn_direction="right")
            possible_headings = [(pose.heading + angle//45) % 8 for _, angle in possible_angles ]
            # Reading initial magnetometer reading
            # initial_mag = time.time(), behaviors.adc.readangle()
            # Store current pose values before turning
            pose0 = pose.clone()
            # returning the turn angle and turn time
            turnAngle, turn_time = behaviors.turn_to_next_street("right")
            time_turn_estimate = -1 * turn_fit(turn_time)
            # print(f'time turn estimate: {time_turn_estimate} degrees')
            print(f'Turn time: {turn_time}\t time-based turn:{time_turn_estimate}\t mag-based turn:{turnAngle}\t list of angles:{possible_headings}')
            chosen_angle, chosen_offset = choose_best_angle(time_turn_estimate, turnAngle, [angle for _, angle in possible_angles], pose)
            # print(f'Chosen angle: {chosen_angle}')
            if not num_streets_to_goal[1]:
                pose.calcturn(chosen_angle, True)
            else:
                pose.heading = num_streets_to_goal[1].pop(0)
            intersection = map.getintersection(pose.x, pose.y)
            # turned_angle = abs(turnAngle)
            turned_angle = abs(chosen_angle)
            dh = (pose0.heading - pose.heading) % 8
            da_lower = (dh + 1) * 45
            da_upper = (dh - 1) * 45
            if intersection.streets[pose.heading] == STATUS.NONEXISTENT:
                if abs(turned_angle - da_lower) < abs(turned_angle - da_upper):
                    street = intersection.streets[(pose.heading + 1) % 8] 
                    # Only adjust heading if the next closest street is already explored (CONNECTED or DEADEND)
                    if street in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.UNEXPLORED]:
                        pose.heading = (pose.heading + 1) % 8
                        print("BINGAAAAA")
                else:
                    street = intersection.streets[(pose.heading - 1) % 8] 
                    if street in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.UNEXPLORED]:
                        pose.heading = (pose.heading - 1) % 8
                        print("BINGBBBBB")
            map.outcomeA(pose0, pose, False)
            time.sleep(0.2)
            behaviors.realign()
            if shared.acquire():
                try:
                    shared.command = None
                finally:
                    shared.release()

        elif command == 'straight':
            current = map.getintersection(pose.x, pose.y)
            current_heading = pose.heading

            # 8.2a: Before driving down a street, always double check for blockage
            if current_heading % 2 == 0:
                # check_distance = .65
                check_distance = .35
            else:
                # check_distance = .90
                check_distance = .7
            blocked = behaviors.check_blockage(distance=check_distance)
            if blocked: 
                print("Cannot go straight: Double check revealed blockage!")
                if shared.acquire():
                    try:
                        shared.command = None
                    finally:
                        shared.release()
                # maybe fix this for edge case when we are at an intersection and try to move forward
                # but it marks it as blocked instead of waiting for blockage to leave
                map.getintersection(pose.x, pose.y).updateStreet(current_heading, STATUS.BLOCKED)
                map.plot(pose)
                # If we have a goal, recalculate path
                if goal is not None:
                    print("Recalculating path to goal after encountering blocked street...")
                    map.setstreet(goal[0], goal[1])
                continue 

            if current.streets[current_heading] in [STATUS.NONEXISTENT, STATUS.BLOCKED]:
                print("Cannot go straight: no street ahead or known blockage!")
                if shared.acquire():
                    try:
                        shared.command = None
                    finally:
                        shared.release()
                # If we have a goal, recalculate path
                if goal is not None:
                    print("Recalculating path to goal after encountering blocked street...")
                    map.setstreet(goal[0], goal[1])
                continue

            print("Going Straight")
            # Store current pose values before moving
            pose0 = pose.clone()
            isUturn, travel_time, road_ahead = behaviors.line_follow()
            # Outcome B
            if not isUturn:
                pose.calcmove()
                map.outcomeB(pose0, pose, road_ahead)
            # Outcome C
            else:
                pose.calcuturn()
                map.outcomeC(pose0, pose, road_ahead)
            if shared.acquire():
                try:
                    shared.command = None
                finally:
                    shared.release()

            if (pose.x, pose.y) == goal:
                # If was in goal mode, go to manual
                # If was in autonomous mode, clear goal
                if shared.acquire():
                    try:
                        if shared.mode == 2:
                            shared.mode = 0
                            print("Goal reached! Returning to manual mode from goal mode")
                        elif shared.mode == 1:
                            print("Goal reached! Continuing autonomous mode")
                        elif shared.mode == 3 and before_pause == 2:
                            print("Goal reached after the final step! Returning to manual mode from goal mode")
                            shared.mode = 0
                        shared.goal = None
                    finally:
                        shared.release()
                map.setstreet(None, None)  # Clear optimal path tree

        # # maybe change this for checking blocked streets
        # is_blocked = behaviors.check_blockage()
        # map.check_and_set_blocked(pose, is_blocked)
        # Update visualization after each action
        if shared.acquire():
            # Copy the robot’s current pose.
            shared.robotx = copy.deepcopy(pose.x) 
            shared.roboty = copy.deepcopy(pose.y) 
            shared.robotheading = copy.deepcopy(pose.heading)
            shared.release()
        map.plot(pose)


def main_simple_brain():
    """
    Main entry point for the robot navigation and mapping program.

    This function:
    1. Parses command line arguments
    2. Initializes hardware components (pigpio, drive system, sensors)
    3. Creates a map instance
    4. Runs the simple_brain control loop
    5. Handles cleanup and exceptions

    Command line arguments:
        --display: Enable real-time map display using TkAgg backend
        --x: Initial x-coordinate (default: 0.0)
        --y: Initial y-coordinate (default: 0.0)
        --heading: Initial heading direction (0-7, default: 0)
    """
    from ui_main import runui

    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Robot navigation and mapping")
    parser.add_argument(
        "--display",
        action="store_true",
        help="Display map in real-time using TkAgg backend",
    )
    parser.add_argument(
        "--x",
        type=float,
        default=0.0,
        help="Initial x-coordinate (default: 0.0)",
    )
    parser.add_argument(
        "--y",
        type=float,
        default=0.0,
        help="Initial y-coordinate (default: 0.0)",
    )
    parser.add_argument(
        "--heading",
        type=int,
        default=0,
        help="Initial heading direction (0-7, default: 0)",
    )
    args = parser.parse_args()

    # Set environment variable for map display
    if args.display:
        os.environ["display_map"] = "on"
        print("Display map enabled - using TkAgg backend")

    # Initialize the pigpio interface
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon. Is it running?")
        return

    robot = Robot(io)
    drive_system = DriveSystem(io)
    sensors = LineSensor(io)
    adc = ADC(io)
    proximity_sensor = ProximitySensor(io)
    behaviors = Behaviors(drive_system, sensors, adc, proximity_sensor)

    shared = SharedData()
    # Start the ROS worker thread.
    rosthread = threading.Thread(name="ROSThread", target=runros, args=(shared,))
    rosthread.start()
    
    ui_thread = threading.Thread(target=runui, args=(shared,))
    ui_thread.daemon = True
    ui_thread.start()

    try:
        simple_brain(behaviors, robot, shared, args.x, args.y, args.heading)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down...")
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally:
        try:
            # Shutdown cleanly only if still connected
            if io.connected:
                robot.stop()
            # End the ROS thread (send the KeyboardInterrupt exception).
            ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(rosthread.ident), ctypes.py_object(KeyboardInterrupt))
            rosthread.join()
        except Exception:
            # If any exception occurs during cleanup, just print it
            print("Error during cleanup")
            traceback.print_exc()


if __name__ == "__main__":
    main_simple_brain()
