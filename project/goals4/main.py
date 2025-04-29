# Imports
import pigpio
import traceback
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


def simple_brain(behaviors, robot):

    pose = Pose()
    map = Map(pose.x, pose.y, pose.heading)

    # map.plot(pose.x, pose.y, pose.heading)

    while True:
        try:
            cmd = input("Enter command (s, l, r, q): ").strip().lower()

        except KeyboardInterrupt:
            robot.stop()
            break

        if cmd == "q":
            print("Quitting...")
            robot.stop()
            break

        ## OUTCOME A ##
        elif cmd == "l":
            # perform left turn
            print("Turning left...")

            # Store current pose values before turning
            x0, y0, h0 = pose.x, pose.y, pose.heading
            
            angle1, angle2 = behaviors.turn_to_next_street("left")
            pose.calcturn(angle1, angle2)
            
            # Get new pose values after turning
            x1, y1, h1 = pose.x, pose.y, pose.heading

            map.outcomeA(x0, y0, h0, x1, y1, h1, True)

        elif cmd == "r":
            # perform right turn
            print("Turning right...")

            # Store current pose values before turning
            x0, y0, h0 = pose.x, pose.y, pose.heading
            
            angle1, angle2 = behaviors.turn_to_next_street("right")
            pose.calcturn(angle1, angle2)
            
            # Get new pose values after turning
            x1, y1, h1 = pose.x, pose.y, pose.heading

            map.outcomeA(x0, y0, h0, x1, y1, h1, False)

        ## OUTCOME B + C ##
        elif cmd == "s":
            print("Going Straight")

            # Store current pose values before moving
            x0, y0, h0 = pose.x, pose.y, pose.heading
            
            isUturn, travel_time = behaviors.line_follow()

            # Outcome B
            if not isUturn:
                pose.calcmove()
                # Get new pose values after moving
                x1, y1, h1 = pose.x, pose.y, pose.heading
                map.outcomeB(x0, y0, h0, x1, y1, h1)
            # Outcome C
            else:
                pose.calcuturn()
                # Get new pose values after U-turn
                x1, y1, h1 = pose.x, pose.y, pose.heading
                map.outcomeC(x0, y0, h0, x1, y1, h1)


        else:
            print("Invalid command, quitting...")
            robot.stop()
            break

def main_simple_brain():
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
        simple_brain(behaviors, robot)
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
