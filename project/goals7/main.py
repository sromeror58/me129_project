import pigpio
import traceback
import os
import argparse
import threading
import time
import copy
from drive_system import DriveSystem, turn_fit
from sensor import LineSensor
from behaviors import Behaviors
from pose import Pose
from magnetometer import ADC
from map import Map, Intersection, STATUS
from proximitysensor import ProximitySensor

class SharedData:
    """
    Thread-safe shared data structure for robot control and UI communication.
    """
    def __init__(self):
        self._lock = threading.Lock()

        self.mode = 0  # 0: manual, 1: autonomous, 2: goal-seeking, 3: paused
        self.step_mode = False  # For step functionality
        self.paused = False  # For pause functionality

        self.pose = None  # Current robot pose
        self.goal = None  # (x, y) coordinates

        self.command = None  # Current command to execute

        self.map_filename = None  # For load/save operations
        self.map = None  # The current map state

        self.running = True  # Control flag for thread termination
        self.error = None  # For error reporting

        self.num_streets_to_goal = [0, []]  # Track turns needed to get to goal

    def update(self, **kwargs):
        """Thread-safe update of multiple attributes"""
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self, key):
                    setattr(self, key, value)

    def get(self, *attrs):
        """Thread-safe retrieval of multiple attributes"""
        with self._lock:
            return tuple(copy.deepcopy(getattr(self, attr)) for attr in attrs)

    def set(self, **kwargs):
        """Thread-safe setting of multiple attributes"""
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self, key):
                    setattr(self, key, copy.deepcopy(value))

class Robot:
    """
    Class that encapsulates all the robot components such as the drive system
    and sensors.
    """
    def __init__(self, io):
        self.io = io
        self.drive_system = DriveSystem(io)
        self.sensors = LineSensor(io)
        self.adc = ADC(io)
        self.proximity_sensor = ProximitySensor(io)
        self.behaviors = Behaviors(self.drive_system, self.sensors, self.adc, self.proximity_sensor)

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

def choose_best_angle(time_estimate, mag_estimate, headings):
    weight_time = 0.4
    weight_mag = 0.6
    if abs(mag_estimate) <= 180 or abs(time_estimate) <= 180:
        weight_time = 0.7
        weight_mag = 0.3
    weighted_average = weight_time * time_estimate + weight_mag * mag_estimate
    print(weighted_average)
    # weighted_average = (weight_time * time_estimate + weight_mag * mag_estimate) % 360
    best_offset = min(headings, key=lambda head: abs(weighted_average - head * 45))
    best_angle = best_offset * 45
    print(best_angle, best_offset)
    return best_angle, best_offset

def fullrobot(shared):
    """
    Main robot control function running in its own thread.
    Handles all robot operations and state management.
    """
    try:
        # Initialize hardware
        io = pigpio.pi()
        if not io.connected:
            shared.update(error="Failed to connect to pigpio daemon")
            return

        robot = Robot(io)
        map_obj, pose = initialize_map(robot.behaviors, robot, 0, 0, 0)
        
        # Initialize shared data
        shared.set(
            pose=pose,
            map=map_obj,
            mode=0,  # Start in manual mode
            running=True,
            paused=False
        )

        while shared.get('running')[0]:
            try:
                # Get current state
                mode, command, goal, step_mode, paused = shared.get('mode', 'command', 'goal', 'step_mode', 'paused')
                
                if step_mode and not command:
                    time.sleep(0.1)  # Small delay in step mode
                    continue

                # Handle different modes
                if mode == 0:  # Manual mode
                    if command:
                        # Process command
                        if command == 'quit':
                            shared.update(running=False)
                            break
                        elif command == 'save':
                            if shared.get('map_filename')[0]:
                                shared.get('map')[0].save_map(filename=shared.get('map_filename')[0])
                        elif command == 'load':
                            if shared.get('map_filename')[0]:
                                try:
                                    new_map = Map.load_map(shared.get('map_filename')[0])
                                    # Preserve current pose when loading map
                                    current_pose = shared.get('pose')[0]
                                    shared.update(map=new_map, pose=current_pose)
                                    print("Map loaded successfully")
                                except Exception as e:
                                    print(f"Error loading map: {e}")
                        elif command == 'goal':
                            if goal:
                                map_obj = shared.get('map')[0]
                                if goal not in map_obj.intersections:
                                    print("Invalid goal coordinates. Goal must be an explored intersection.")
                                else:
                                    print(f"Setting goal to {goal}")
                                    map_obj.setstreet(goal[0], goal[1])
                                    map_obj.plot(pose)
                                    shared.update(mode=2)  # Switch to goal-seeking mode
                        elif command == 'pose':
                            new_pose = shared.get('new_pose')[0]
                            if new_pose:
                                x, y, heading = new_pose
                                pose.x = x
                                pose.y = y
                                pose.heading = heading
                                shared.update(pose=pose)
                                map_obj.plot(pose)
                        elif command == 'show':
                            map_obj.plot(pose)
                        elif command == 'explore':
                            shared.update(mode=1, paused=False)  # Switch to autonomous mode
                            print("Starting autonomous exploration")
                        elif command == 'pause':
                            shared.update(paused=True)
                            print("Pausing after next intersection")
                        elif command == 'step':
                            if paused:
                                # Only step if we're at an intersection
                                current = map_obj.getintersection(pose.x, pose.y)
                                if current is not None:
                                    shared.update(step_mode=True)
                                    print("Taking one step")
                                else:
                                    print("Cannot step: not at an intersection")
                        elif command == 'resume':
                            if paused:
                                shared.update(paused=False, step_mode=False)
                                print("Resuming autonomous movement")
                        elif command in ['left', 'right', 'straight']:
                            # Execute movement command
                            execute_movement(command, robot, shared)
                        shared.update(command=None)  # Clear command after processing

                elif mode == 1:  # Autonomous mode
                    if paused:
                        if step_mode:
                            # Only execute step if we're at an intersection
                            current = map_obj.getintersection(pose.x, pose.y)
                            if current is not None:
                                autonomous_exploration(robot, shared)
                                shared.update(step_mode=False)
                            else:
                                print("Cannot step: not at an intersection")
                                shared.update(step_mode=False)
                        time.sleep(0.1)
                    else:
                        autonomous_exploration(robot, shared)

                elif mode == 2:  # Goal-seeking mode
                    if paused:
                        if step_mode:
                            # Only execute step if we're at an intersection
                            current = map_obj.getintersection(pose.x, pose.y)
                            if current is not None:
                                goal_seeking(robot, shared)
                                shared.update(step_mode=False)
                            else:
                                print("Cannot step: not at an intersection")
                                shared.update(step_mode=False)
                        time.sleep(0.1)
                    else:
                        goal_seeking(robot, shared)

                time.sleep(0.1)  # Prevent CPU hogging

            except Exception as e:
                shared.update(error=f"Error in robot control: {str(e)}")
                traceback.print_exc()

    except Exception as e:
        shared.update(error=f"Fatal error in robot thread: {str(e)}")
        traceback.print_exc()
    finally:
        if 'robot' in locals():
            robot.stop()

def runui(shared):
    """
    UI thread function handling user input and display.
    """
    try:
        while shared.get('running')[0]:
            try:
                cmd = input("Enter command (explore/goal/pause/step/resume/left/right/straight/save/load/pose/show/quit): ").strip().lower()
                
                if cmd == 'quit':
                    shared.update(running=False)
                    break
                elif cmd == 'save':
                    filename = input("Name to save map to: ")
                    print("Saving plot...")
                    shared.update(map_filename=filename, command='save')
                elif cmd == 'load':
                    filename = input("Enter filename to load: ")
                    shared.update(map_filename=filename, command='load')
                elif cmd == 'goal':
                    try:
                        x = int(input("Enter goal x-coordinate: "))
                        y = int(input("Enter goal y-coordinate: "))
                        shared.update(goal=(x, y), command='goal')
                    except ValueError:
                        print("Invalid input. Please enter integer coordinates.")
                elif cmd == 'pose':
                    try:
                        x = int(input("Enter current x-coordinate: "))
                        y = int(input("Enter current y-coordinate: "))
                        heading = int(input("Enter current heading (0-7): "))
                        if 0 <= heading <= 7:
                            shared.update(command='pose', new_pose=(x, y, heading))
                        else:
                            print("Invalid heading. Must be between 0 and 7.")
                    except ValueError:
                        print("Invalid input. Please enter integer coordinates.")
                elif cmd == 'show':
                    shared.update(command='show')
                elif cmd in ['explore', 'pause', 'step', 'resume', 'left', 'right', 'straight']:
                    shared.update(command=cmd)
                else:
                    print("Invalid command...")

            except Exception as e:
                shared.update(error=f"Error in UI thread: {str(e)}")
                traceback.print_exc()

    except Exception as e:
        shared.update(error=f"Fatal error in UI thread: {str(e)}")
        traceback.print_exc()

def execute_movement(cmd, robot, shared):
    """Execute a movement command and update the map accordingly."""
    pose, map_obj = shared.get('pose', 'map')
    pose0 = pose.clone()

    if cmd == 'left':
        print("Turning left...")
        possible_turns = map_obj.get_unexplored_streets(pose.x, pose.y)
        possible_headings = [t[0] for t in possible_turns]
        initial_mag = robot.behaviors.adc.readangle()
        turnAngle, turn_time = robot.behaviors.turn_to_next_street("left")
        time_turn_estimate = turn_fit(turn_time)
        chosen_angle, chosen_offset = choose_best_angle(time_turn_estimate, turnAngle, possible_headings)
        
        if not shared.get('num_streets_to_goal')[0][1]:
            pose.calcturn(chosen_angle, True)
        else:
            pose.heading = shared.get('num_streets_to_goal')[0][1].pop(0)

        map_obj.outcomeA(pose0, pose, True)

    elif cmd == 'right':
        print("Turning right...")
        possible_turns = map_obj.get_unexplored_streets(pose.x, pose.y)
        possible_headings = [t[0] for t in possible_turns]
        initial_mag = robot.behaviors.adc.readangle()
        turnAngle, turn_time = robot.behaviors.turn_to_next_street("right")
        time_turn_estimate = -1 * turn_fit(turn_time)
        chosen_angle, chosen_offset = choose_best_angle(time_turn_estimate, turnAngle, possible_headings)
        
        if not shared.get('num_streets_to_goal')[0][1]:
            pose.calcturn(-1 * chosen_angle, True)
        else:
            pose.heading = shared.get('num_streets_to_goal')[0][1].pop(0)

        map_obj.outcomeA(pose0, pose, False)

    elif cmd == 'straight':
        current = map_obj.getintersection(pose.x, pose.y)
        if current.streets[pose.heading] == STATUS.NONEXISTENT:
            print("Cannot go straight: no street ahead!")
            return

        print("Going Straight")
        isUturn, travel_time, road_ahead = robot.behaviors.line_follow()

        if not isUturn:
            pose.calcmove()
            map_obj.outcomeB(pose0, pose, road_ahead)
        else:
            pose.calcuturn()
            map_obj.outcomeC(pose0, pose, road_ahead)

    shared.update(pose=pose, map=map_obj)
    map_obj.plot(pose)

def autonomous_exploration(robot, shared):
    """Handle autonomous exploration mode."""
    pose, map_obj = shared.get('pose', 'map')
    
    # In the process of making an accurate turn
    if shared.get('num_streets_to_goal')[0][1]:
        if shared.get('num_streets_to_goal')[0][0] > 0:
            shared.update(command='left')  # Turn left
        else:
            shared.update(command='right')  # Turn right
        map_obj.plot(pose)  # Update visualization
        return

    # Get unexplored streets and find the one closest to current heading
    unexplored_streets = map_obj.get_unexplored_streets(pose.x, pose.y)
    if not unexplored_streets:
        nearest = map_obj.find_nearest_unexplored(pose.x, pose.y)
        if nearest is None:
            print("Map fully explored! Returning to manual mode.")
            shared.update(mode=0)
            map_obj.plot(pose)  # Update visualization
            return
        
        shared.update(goal=nearest, mode=2)
        map_obj.plot(pose)  # Update visualization
        return

    closest_heading = min(unexplored_streets, 
                         key=lambda x: min((x[0] - pose.heading) % 8, 
                                         (pose.heading - x[0]) % 8))[0]

    if closest_heading == pose.heading:
        shared.update(command='straight')
    else:
        # Local exploration - choose first unexplored street
        heading_diff = (closest_heading - pose.heading) % 8

        # Can we make an accurate turn?
        # If all the streets are known between the current heading and target heading, then we can make the turn more accurate
        current_heading = pose.heading
        if heading_diff <= 4:
            num_streets_to_goal = [1, []]  # Left turns
        else:
            num_streets_to_goal = [-1, []]  # Right turns

        # Check all streets between current heading and target heading
        current = map_obj.getintersection(pose.x, pose.y)
        while current_heading != closest_heading:
            current_heading = (current_heading + num_streets_to_goal[0]) % 8
            # If any street is UNKNOWN, we can't make an accurate turn
            if current.streets[current_heading] == STATUS.UNKNOWN:
                num_streets_to_goal[1] = []  # Clear the streets to encounter
                break
            # Otherwise, if the street EXISTS, then we should take note of it
            elif current.streets[current_heading] != STATUS.NONEXISTENT:
                num_streets_to_goal[1].append(current_heading)

        # Now we can make an accurate turn, if it exists; otherwise, a less accurate turn
        if heading_diff <= 4:
            shared.update(command='left', num_streets_to_goal=num_streets_to_goal)
        else:
            shared.update(command='right', num_streets_to_goal=num_streets_to_goal)
    
    map_obj.plot(pose)  # Update visualization

def goal_seeking(robot, shared):
    """Handle goal-seeking mode."""
    pose, map_obj, goal = shared.get('pose', 'map', 'goal')
    
    if (pose.x, pose.y) == goal:
        print("Goal reached! Returning to manual mode.")
        shared.update(mode=0, goal=None)
        map_obj.setstreet(None, None)
        map_obj.plot(pose)  # Update visualization
        return

    current = map_obj.getintersection(pose.x, pose.y)
    if current.direction is None:
        print("No valid path to goal. Returning to manual mode.")
        shared.update(mode=0, goal=None)
        map_obj.setstreet(None, None)
        map_obj.plot(pose)  # Update visualization
        return

    if pose.heading != current.direction:
        heading_diff = (current.direction - pose.heading) % 8
        shared.update(command='left' if heading_diff <= 4 else 'right')
    else:
        shared.update(command='straight')
    
    map_obj.plot(pose)  # Update visualization

def main():
    """Main entry point for the robot navigation and mapping program."""

    parser = argparse.ArgumentParser(description="Robot navigation and mapping")
    parser.add_argument("--display", action="store_true",
                       help="Display map in real-time using TkAgg backend")
    parser.add_argument("--x", type=float, default=0.0,
                       help="Initial x-coordinate (default: 0.0)")
    parser.add_argument("--y", type=float, default=0.0,
                       help="Initial y-coordinate (default: 0.0)")
    parser.add_argument("--heading", type=int, default=0,
                       help="Initial heading direction (0-7, default: 0)")
    args = parser.parse_args()

    if args.display:
        os.environ["display_map"] = "on"
        print("Display map enabled - using TkAgg backend")

    shared = SharedData()
    
    # Create and start threads
    robot_thread = threading.Thread(target=fullrobot, args=(shared,))
    ui_thread = threading.Thread(target=runui, args=(shared,), daemon=True)
    
    robot_thread.start()
    ui_thread.start()
    
    try:
        # Wait for robot thread to complete
        robot_thread.join()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down...")
        shared.update(running=False)
    finally:
        # Ensure clean shutdown
        if shared.get('error')[0]:
            print(f"Error occurred: {shared.get('error')[0]}")
        
        # Wait for UI thread to finish
        ui_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()
