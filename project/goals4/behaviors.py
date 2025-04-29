import time
from sensor_estimation import (
    IntersectionEstimator,
    SideEstimator,
    EndOfStreetEstimator,
    NextStreetDetector,
)
from config import (
    STRAIGHT,
    TURN_L,
    HOOK_L,
    TURN_R,
    HOOK_R,
    SPIN_L,
    SPIN_R,
)


class Behaviors:
    def __init__(self, drive_system, sensors, adc):
        self.drive_system = drive_system
        self.sensors = sensors
        self.adc = adc
        
    def turn_to_next_street(self, direction):
        """Performs the turning behavior to find the next street at an intersection.

        This behavior:
        1. Spins in the specified direction
        2. Detects when the robot has left the current street
        3. Detects when the robot has found the next street
        4. Stops when aligned with the next street

        Args:
            direction (str): Direction to turn, either 'left' or 'right'
        """
        # Validate and convert the direction parameter
        if direction.lower() == "left":
            spin_direction = SPIN_L
        elif direction.lower() == "right":
            spin_direction = SPIN_R
        else:
            print(f"Invalid direction: {direction}. Using 'left' as default.")
            spin_direction = SPIN_L

        print(f"Turning to next street: {direction}")

        # Initialize the next street detector
        next_street_detector = NextStreetDetector()

        # Continuous loop for turning behavior
        t0 = time.time()
        angle1 = self.adc.readangle()

        while True:
            # Read sensor data
            reading = self.sensors.read()

            if next_street_detector.update(reading, time_constant=0.1):
                curr = time.time()
                angle2 = self.adc.readangle()
                print(f"Found and aligned with next street!\nTime to turn to next street was {curr - t0} sec.")
                break
            
            else:
                self.drive_system.drive(spin_direction)

        self.drive_system.stop()
        return angle1, angle2
    
    def pull_forward(self, travel_time=0.5):
        """Performs the pull forward behavior after detecting a valid intersection.

        Args:
            travel_time (float): Amount of time to pull forward.
        """
        t_0 = time.time()
        curr = time.time()
        while curr - t_0 <= travel_time:
            self.drive_system.drive(STRAIGHT)
            curr = time.time()
        self.drive_system.stop()
        
    def line_follow(self):
        """Performs the line following behavior.
        
        """
        intersection_estimator = IntersectionEstimator()
        side_estimator = SideEstimator()
        eos_estimator = EndOfStreetEstimator()
        
        t0 = time.time()
        while True:
            reading = self.sensors.read()

            # Check for intersection
            if intersection_estimator.update(reading, 0.1):
                curr = time.time()
                # Then pull forward
                self.pull_forward(travel_time=0.4)

                # isUturn, travel time
                return False, curr - t0

            # Estimate which side of the road the robot is on
            side = side_estimator.update(reading, 0.05)
            print(f"Road side: {side}")

            # Check for end of street
            if eos_estimator.update(reading, side, 0.6):
                curr = time.time()
                print("End of street detected!")
                self.turn_to_next_street("left")
                self.line_follow()
                self.drive_system.stop()

                # isUturn, travel time of street, one way
                return True, curr - t0

            if reading == (0, 1, 0):
                self.drive_system.drive(STRAIGHT)
            elif reading == (1, 1, 1):
                self.drive_system.drive(STRAIGHT)
            elif reading == (0, 1, 1):
                self.drive_system.drive(TURN_R)
            elif reading == (0, 0, 1):
                self.drive_system.drive(HOOK_R)
            elif reading == (1, 1, 0):
                self.drive_system.drive(TURN_L)
            elif reading == (1, 0, 0):
                self.drive_system.drive(HOOK_L)
            elif reading == (0, 0, 0):
                # When all sensors read 0, use the road side estimator to decide what to do
                if side == SideEstimator.LEFT:
                    # If pushed to the left, turn or hook right to get back to center
                    self.drive_system.drive(HOOK_R)
                elif side == SideEstimator.RIGHT:
                    # If pushed to the right, turn or hook left to get back to center
                    self.drive_system.drive(HOOK_L)
                else:
                    # If centered, keep going straight
                    self.drive_system.drive(STRAIGHT)
            else:
                # considering the other 3 cases i.e. 101, 111, and 000
                self.drive_system.stop()
