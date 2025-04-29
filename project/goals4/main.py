# Imports
import pigpio
import traceback
import os
import sys
import argparse
from drive_system import DriveSystem
from sensor import LineSensor
from behaviors import Behaviors
import time
from pose import Pose
from magnetometer import ADC
from map import Map

class Robot:
    """
    Class that encapsulates all the robot components such as the drive system
    and sensors.
    """

    def __init__(self, io):
        """Initializes the motors, drive system and sensors.

        Args:
            io (pigpio.pi): pigpio interface instance
        """
        self.io = io
        self.drive_system = DriveSystem(io)
        self.sensors = LineSensor(io)

    def stop(self):
        """Safely stop all motors and clean up the pigpio connection"""
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


def simple_brain(behaviors, robot, map):
    """
    Simple brain function that handles robot navigation and mapping.
    
    Args:
        behaviors (Behaviors): Robot behaviors
        robot (Robot): Robot instance
        map (Map): Map instance
    """

    pose = Pose()
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
            
            angle1, angle2 = behaviors.turn_to_next_street("left")
            pose.calcturn(angle1, angle2)
            
            map.outcomeA(pose0, pose, True)

        elif cmd == "r":
            # perform right turn
            print("Turning right...")

            # Store current pose values before turning
            pose0 = pose.clone()
            
            angle1, angle2 = behaviors.turn_to_next_street("right")
            pose.calcturn(angle1, angle2)
            
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
            print("Invalid command, quitting...")
            robot.stop()
            map.close()  
            break

def main_simple_brain():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Robot navigation and mapping')
    parser.add_argument('--display', action='store_true', help='Display map in real-time using TkAgg backend')
    args = parser.parse_args()
    
    # Set environment variable for map display
    if args.display:
        os.environ['display_map'] = 'on'
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
    map = None

    try:
        # Create a map instance and pass it to simple_brain
        map = Map()
        simple_brain(behaviors, robot, map)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally:
        try:
            # Close matplotlib resources if map was created
            if map is not None:
                map.close()
                
            # Shutdown cleanly only if still connected
            if io.connected:
                robot.stop()
        except Exception:
            # If any exception occurs during cleanup, just print it
            print("Error during cleanup")
            traceback.print_exc()


if __name__ == "__main__":
    main_simple_brain()
