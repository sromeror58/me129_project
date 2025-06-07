import time
from sensor_estimation import (
    IntersectionEstimator,
    SideEstimator,
    EndOfStreetEstimator,
    NextStreetDetector,
    StreetDetector,
)
from pose import getTurnAngle
from config import (
    STRAIGHT,
    TURN_L,
    HOOK_L,
    TURN_R,
    HOOK_R,
    SPIN_L,
    SPIN_R,
    STEER_R,
    STEER_L
)


class Behaviors:
    """
    Class that encapsulates the robot's high-level behaviors.

    This class provides methods for executing complex robot behaviors such as
    turning, line following, and intersection handling. It coordinates between
    the drive system, sensors, and magnetometer to implement these behaviors.
    """

    def __init__(self, drive_system, sensors, adc, proximity_sensor, nfc_sensor):
        """
        Initialize the Behaviors class with the necessary components.

        Args:
            drive_system: The robot's drive system for motor control
            sensors: The robot's sensors for environmental perception
            adc: The analog-to-digital converter for magnetometer readings
            proximity_sensor: The robot's proximity sensor for obstacle detection
            nfc_sensor: The robot's NFC Sensor for identifying it's current Intersection ID
        """
        self.drive_system = drive_system
        self.sensors = sensors
        self.adc = adc
        self.proximity_sensor = proximity_sensor
        self.nfc_sensor = nfc_sensor

    def turn_to_next_street(self, direction):
        """
        Performs the turning behavior to find the next street at an intersection.

        This behavior:
        1. Spins in the specified direction
        2. Detects when the robot has left the current street
        3. Detects when the robot has found the next street
        4. Stops when aligned with the next street
        5. Calculates the turn angle based on magnetometer readings

        Args:
            direction (str): Direction to turn, either 'left' or 'right'

        Returns:
            tuple:
                - float: The calculated turn angle in degrees, based on magnetometer data
                - float: Duration in seconds taken to complete the turn
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
        adjustedangle1 = None

        while True:
            # Read sensor data
            reading = self.sensors.read()
            # print('yo')
            state, isTransition = next_street_detector.update(
                reading, time_constant=0.07
            )

            if isTransition:
                adjustedangle1 = self.adc.readangle()
                print("BING")

            if state:
                curr = time.time()
                isLeft = False
                if spin_direction == SPIN_L:
                    isLeft = True
                print(
                    f"Found and aligned with next street!\nTime to turn to next street was {curr - t0} sec."
                )
                break

            else:
                self.drive_system.drive(spin_direction)

        self.drive_system.stop()

        turnAngle1 = getTurnAngle(angle1, self.adc.readangle(), isLeft)
        # print(f'Turn angle from getTurnAngle: {turnAngle1}')
        # if adjustedangle1:
        #     turnAngle2 = getTurnAngle(adjustedangle1, self.adc.readangle(), isLeft)
        #     print(turnAngle2)

        #     # If less than full turn, since full turn's are pretty obvious
        #     if abs(turnAngle2 - turnAngle1) <= 90:
        #         # Want to weight closer to turnAngle2 the larger the turn is: 0/5 to 4/5
        #         x = abs(turnAngle1) / 360
        #         weight = (3 * x**2 - 2 * x**3) / 1.15
        #         turnAngle1 = (1 - weight) * turnAngle1 + (weight) * turnAngle2

        # print(turnAngle1)
        return turnAngle1, curr - t0

    def realign(self):
        """
        Performs realignment after turning.
        """
        
        while True:
            reading = self.sensors.read()
            if reading == (0, 1, 0):
                self.drive_system.stop()
                break
            elif reading == (0, 1, 1):
                self.drive_system.drive(SPIN_R)
            elif reading == (0, 0, 1):
                self.drive_system.drive(SPIN_R)
            elif reading == (1, 1, 0):
                self.drive_system.drive(SPIN_L)
            elif reading == (1, 0, 0):
                self.drive_system.drive(SPIN_L)
        

    def pull_forward(self, travel_time=0.55):
        """
        Performs the pull forward behavior after detecting a valid intersection.

        This method drives the robot straight ahead for a specified duration,
        typically used to center the robot in an intersection or move past
        a detected intersection point.

        Args:
            travel_time (float): Amount of time in seconds to pull forward.
                               Defaults to 0.5 seconds.
        Returns:
            tuple:
                - bool: True if road still exists ahead, False otherwise
                - NFC ID: The detected intersection ID from the NFC sensor, or None if not available
        """
        # self.drive_system.stop()
        # time.sleep(0.2)
        street_detector = StreetDetector(time_constant=0.40)
        t_0 = time.time()
        curr = time.time()
        # reading the intersection ID
        nfc_id = None
        
        while curr - t_0 <= travel_time:
            intersection_id = self.nfc_sensor.read()
            if intersection_id:
                nfc_id = intersection_id
            self.drive_system.drive(STRAIGHT)
            curr = time.time()
            # This means that there is still road that the sensor is picking up
            readings = 1.0 if sum(self.sensors.read()) >= 1.0 else 0.0
            street_detector.update(readings)
        self.drive_system.stop()
        readings = 1.0 if sum(self.sensors.read()) >= 1.0 else 0.0
        print(f'road state: {street_detector.update(readings)}')
        return street_detector.update(readings), nfc_id

    def check_blockage(self, distance: float = 0.4):
        """
        Checks if the street ahead is blocked using the front-facing ultrasound sensor.
        A street is considered blocked if there is an obstacle within 70cm.

        Returns:
            bool: True if the street ahead is blocked, False otherwise
        """
        # Read the middle (front-facing) ultrasound sensor
        _, middle_distance, _ = self.proximity_sensor.read()
        
        # If no reading or reading is too close, consider it blocked
        if middle_distance is None or middle_distance < distance:  
            # print(f"Street ahead is blocked! Distance: {middle_distance}m")
            return True
            
        # print(f"Street ahead is clear. Distance: {middle_distance}m")
        return False

    def line_follow(self):
        """
        Performs the line following behavior.

        This method implements the core line-following algorithm that:
        1. Continuously reads sensor data to determine the robot's position relative to the line
        2. Detects intersections and handles them appropriately
        3. Detects the end of streets and initiates U-turns
        4. Adjusts the robot's direction based on sensor readings to stay on the line
        5. Detects and handles moving obstacles by stopping and resuming when clear
        
        Returns:
            tuple (isUturn, travel_time, road_ahead, intersection_id) where:
                - isUturn (bool): True if a U-turn was performed, False otherwise
                - travel_time (float): Time spent following the line in seconds
                - road_ahead (bool): whether there is a road ahead after pulling forward
                - intersection_id (NFC ID): The detected intersection ID from the NFC sensor, or None if not available
        """
        intersection_estimator = IntersectionEstimator()
        side_estimator = SideEstimator()
        eos_estimator = EndOfStreetEstimator()

        # Obstacle detection thresholds (in meters)
        STOP_THRESHOLD = 0.15  # Stop when obstacle is within 15cm
        RESUME_THRESHOLD = 0.25  # Resume when obstacle is beyond 25cm
        
        t0 = time.time()
        is_stopped = False  # Track if robot is currently stopped due to obstacle

        while True:
            reading = self.sensors.read()
            
            # Check for obstacles ahead using proximity sensor
            _, middle_distance, _ = self.proximity_sensor.read()
            
            if middle_distance is not None:
                if middle_distance < STOP_THRESHOLD and not is_stopped:
                    # Stop the robot when obstacle detected
                    print(f"Obstacle detected at {middle_distance:.2f}m - stopping")
                    self.drive_system.stop()
                    is_stopped = True
                elif middle_distance > RESUME_THRESHOLD and is_stopped:
                    # Resume driving when obstacle clears
                    print(f"Obstacle cleared at {middle_distance:.2f}m - resuming")
                    is_stopped = False

            # Only update detectors and drive if not stopped
            if not is_stopped:
                # Check for intersection
                if intersection_estimator.update(reading, 0.20):
                    curr = time.time()
                    # Then pull forward
                    road_state, intersection_id = self.pull_forward(travel_time=0.45)
                    return False, curr - t0, road_state, intersection_id

                # Estimate which side of the road the robot is on
                side = side_estimator.update(reading, 0.05)

                # Check for end of street
                if eos_estimator.update(reading, side, 0.12):
                    # intersection id could be wrong here 
                    road_state, _ = self.pull_forward(travel_time=0.6)
                    curr = time.time()
                    print("End of street detected!")
                    self.turn_to_next_street("left")
                    _, _, road_state, intersection_id = self.line_follow()
                    self.drive_system.stop()
                    return True, curr - t0, road_state, intersection_id

                # Normal line following behavior
                if reading == (0, 1, 0):
                    self.drive_system.drive(STRAIGHT)
                elif reading == (1, 1, 1):
                    self.drive_system.drive(STRAIGHT)
                elif reading == (0, 1, 1):
                    # self.drive_system.drive(TURN_R)
                    self.drive_system.drive(STEER_R)
                elif reading == (0, 0, 1):
                    # self.drive_system.drive(HOOK_R)
                    self.drive_system.drive(TURN_R)
                elif reading == (1, 1, 0):
                    # self.drive_system.drive(TURN_L)
                    self.drive_system.drive(STEER_L)
                elif reading == (1, 0, 0):
                    # self.drive_system.drive(HOOK_L)
                    self.drive_system.drive(TURN_L)
                elif reading == (0, 0, 0):
                    # print('HELP!!!!!')
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
                elif reading == (1, 0, 1):
                    self.drive_system.drive(STRAIGHT)
                else:
                    # considering the other 3 cases i.e. 111, and 000
                    self.drive_system.stop()
