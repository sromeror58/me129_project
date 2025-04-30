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
from map import Map


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
    pose = Pose(x, y, heading)
    map = Map(pose)
    map.plot(pose)

    while True:
        try:
            cmd = input("Enter command (s, l, r, q, p): ").strip().lower()

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
            print("Saving plot...")
            map.save_plot(pose)

        ## OUTCOME A ##
        elif cmd == "l":
            # perform left turn
            print("Turning left...")

            # Store current pose values before turning
            pose0 = pose.clone()

            turnAngle = behaviors.turn_to_next_street("left")
            pose.calcturn(turnAngle, True)

            map.outcomeA(pose0, pose, True)

        elif cmd == "r":
            # perform right turn
            print("Turning right...")

            # Store current pose values before turning
            pose0 = pose.clone()

            turnAngle = behaviors.turn_to_next_street("right")
            pose.calcturn(turnAngle, False)

            map.outcomeA(pose0, pose, False)

        ## OUTCOME B + C ##
        elif cmd == "s":
            print("Going Straight")

            # Store current pose values before moving
            pose0 = pose.clone()

            isUturn, travel_time = behaviors.line_follow()

            # Outcome B
            if not isUturn:
                pose.calcmove()
                map.outcomeB(pose0, pose)
            # Outcome C
            else:
                pose.calcuturn()
                map.outcomeC(pose0, pose)

        else:
            print("Invalid command...")
            continue


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
