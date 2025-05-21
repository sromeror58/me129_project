 # Imports
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


def initialize_map(behaviors, robot, heading, x, y):
    """
    Asks the user to load a map or create a new one, then returns the map and starting pose.
    """
    choice = input("Load existing map? (y/n): ").strip().lower()
    if choice == "y":
        # only querying file name without extension (i.e. without .pickle at the end for simplicity)
        filename = input("Enter filename (default: mymap.pickle): ").strip() or "mymap"
        map_obj = Map.load_map(filename)
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
        map = Map(pose, intersection_dictionary)
        return map, pose


def simple_brain(behaviors, robot, x=0.0, y=0.0, heading=0):
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
        x (float): Initial x-coordinate (default: 0.0)
        y (float): Initial y-coordinate (default: 0.0)
        heading (int): Initial heading direction (0-7, default: 0)
    """
    map, pose = initialize_map(behaviors, robot, heading, x, y)
    map.plot(pose)
    goal = None  # Track current goal coordinates
    num_streets_to_goal = [0, []] # Track turns needed to get to goal
    autonomous_mode = False

    while True:
        try:
            
            # If we have a goal, check if we've reached it
            if goal is not None:
                if (pose.x, pose.y) == goal:
                    print("Goal reached! Returning to manual mode.")
                    goal = None
                    map.setstreet(None, None)  # Clear optimal path tree
                    map.plot(pose)
                    continue

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
                            cmd = "l"  # Turn left
                        else:
                            cmd = "r"  # Turn right

                    else:
                        print("Going straight along optimal path")
                        cmd = "s"
                else:
                    print("No valid path to goal. Returning to manual mode.")
                    goal = None
                    map.setstreet(None, None)  # Clear optimal path tree
                    map.plot(pose)
                    continue

            elif autonomous_mode:
                # In the process of making an accurate turn
                if num_streets_to_goal[1]:
                    if num_streets_to_goal[0] > 0:
                        cmd = "l"  # Turn left
                    else:
                        cmd = "r"  # Turn right

                else:
                    # Get unexplored streets and find the one closest to current heading
                    unexplored_streets = map.get_unexplored_streets(pose.x, pose.y)
                    closest_heading = min(unexplored_streets, key=lambda x: min((x[0] - pose.heading) % 8, (pose.heading - x[0]) % 8))[0] if unexplored_streets else None
                    print(unexplored_streets)
                    print(closest_heading)


                    if closest_heading is not None:
                        if closest_heading == pose.heading:
                            cmd = "s"

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
                                cmd = "l" 
                            else:
                                cmd = "r" 

                    else:
                        # No unexplored streets here, find nearest unexplored intersection
                        nearest = map.find_nearest_unexplored(pose.x, pose.y)
                        
                        if nearest is None:
                            print("Map fully explored! Returning to manual mode.")
                            autonomous_mode = False
                            continue
                            
                        # Set goal to nearest unexplored intersection
                        goal = nearest
                        map.setstreet(goal[0], goal[1])
                        print(f"Setting goal to unexplored intersection at {goal}")
                        continue  # Let the goal-directed navigation handle the movement

            else:
                # Manual mode - get command from user
                cmd = (
                    input(
                        "Enter command (s=straight, l=left, r=right, g=set goal, a=auto mode, q=quit, p=save): "
                    )
                    .strip()
                    .lower()
                )

        except KeyboardInterrupt:
            robot.stop()
            map.close()
            break

        if cmd == "q":
            print("Quitting...")
            robot.stop()
            map.close()
            break

        # Save plot
        elif cmd == "p":
            name = input("Name to save map to: ")
            print("Saving plot...")
            map.save_map(filename=name)

        # Set goal
        elif cmd == "g":
            try:
                x = int(input("Enter goal x-coordinate: "))
                y = int(input("Enter goal y-coordinate: "))
                if (x, y) not in map.intersections:
                    print(
                        "Invalid goal coordinates. Goal must be an explored intersection."
                    )
                    continue
                goal = (x, y)
                print(f"Setting goal to ({x}, {y})")
                map.setstreet(x, y)
                map.plot(pose)
            except KeyboardInterrupt:
                robot.stop()
                map.close()
                break
            except ValueError:
                print("Invalid input. Please enter integer coordinates.")
                continue

        # Toggle autonomous mode
        elif cmd == "a":
            autonomous_mode = not autonomous_mode
            print(f"Autonomous mode {'enabled' if autonomous_mode else 'disabled'}")
            continue

        ## OUTCOME A ##
        elif cmd == "l":
            # perform left turn
            print("Turning left...")
            # Taking the possible turn angles
            possible_turns = map.get_unexplored_streets(pose.x, pose.y)
            # Reading initial magnetometer reading
            initial_mag = behaviors.adc.readangle()
            # Store current pose values before turning
            pose0 = pose.clone()
            # returning the turn angle and turn time
            turnAngle, turn_time = behaviors.turn_to_next_street("left")
            if not num_streets_to_goal[1]: # If turning without goal-following
                pose.calcturn(turnAngle, True)
            else: # If turning without goal-following
                pose.heading = num_streets_to_goal[1].pop(0)

            intersection = map.getintersection(pose.x, pose.y)

            turned_angle = abs(turnAngle)
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
            # calculating the final time reading and final magnetometer reading
            final_mag = behaviors.adc.readangle()

        elif cmd == "r":
            # perform right turn
            print("Turning right...")
            # Taking the possible turn angles
            possible_turns = map.get_unexplored_streets(pose.x, pose.y)
            # Reading initial magnetometer reading
            initial_mag = time.time(), behaviors.adc.readangle()
            # Store current pose values before turning
            pose0 = pose.clone()
            # returning the turn angle and turn time
            turnAngle, turn_time = behaviors.turn_to_next_street("right")
            if not num_streets_to_goal[1]:
                pose.calcturn(turnAngle, False)
            else:
                pose.heading = num_streets_to_goal[1].pop(0)

            intersection = map.getintersection(pose.x, pose.y)

            turned_angle = abs(turnAngle)
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
            final_mag = behaviors.adc.readangle()

        ## OUTCOME B + C ##
        elif cmd == "s":
            current = map.getintersection(pose.x, pose.y)
            current_heading = pose.heading
            if current.streets[current_heading] == STATUS.NONEXISTENT:
                print("Cannot go straight: no street ahead!")
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
        else:
            print("Invalid command...")
            continue

        # Update visualization after each action
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

    behaviors = Behaviors(drive_system, sensors, adc)

    try:
        # Create a map instance and pass it to simple_brain
        simple_brain(behaviors, robot, args.x, args.y, args.heading)
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
        except Exception:
            # If any exception occurs during cleanup, just print it
            print("Error during cleanup")
            traceback.print_exc()


if __name__ == "__main__":
    main_simple_brain()
