import time

class SensorEstimator:
    """
    Base class that implements a generic sensor-based estimator framework.
    Uses a running average approach followed by thresholding with hysteresis.
    """
    
    def __init__(self, initial_level=0.0, threshold=0.63):
        """
        Initialize the estimator with default values.
        
        Args:
            initial_level (float): Initial level for the running average
            threshold (float): Threshold for state changes (default 0.63, ~one time constant)
        """
        self.level = initial_level
        self.threshold = threshold
        self.tlast = time.time()
    
    def update_level(self, raw_value, time_constant):
        """
        Update the estimator level with a new raw value.
        
        Args:
            raw_value (float): Raw value from sensor reading
            time_constant (float): Time constant for the running average (in seconds)
            
        Returns:
            float: Updated level value
        """
        # Get current time and calculate time step
        tnow = time.time()
        dt = tnow - self.tlast
        self.tlast = tnow
        
        # Running average calculation
        self.level = self.level + dt/time_constant * (raw_value - self.level)
        
        return self.level
    
    def reset(self, level=0.0):
        """
        Reset the estimator to initial values.
        
        Args:
            level (float): Initial level value
        """
        self.level = level
        self.tlast = time.time()


class IntersectionEstimator(SensorEstimator):
    """
    Detector for identifying valid intersections.
    An intersection is defined as a region where all IR sensors detect the line
    and is at least 2.5cm deep in the direction of travel.
    """
    
    def __init__(self, initial_level=0.0, initial_state=False, threshold=0.63):
        """
        Initialize the intersection detector.
        
        Args:
            initial_level (float): Initial level for the running average (0.0-1.0)
            initial_state (bool): Initial state of the detector
            threshold (float): Threshold for state changes (default 0.63, ~one time constant)
        """
        super().__init__(initial_level, threshold)
        self.state = initial_state
    
    def update(self, ir_readings, time_constant=0.5):
        """
        Update the intersection detector with new IR readings.
        
        Args:
            ir_readings (tuple): Tuple of (left, middle, right) IR sensor readings
            time_constant (float): Time constant for the detector (in seconds)
            
        Returns:
            bool: True if a valid intersection is detected, False otherwise
        """
        # Raw guess: 1.0 if all sensors detect the line, 0.0 otherwise
        raw_value = 1.0 if all(ir_readings) else 0.0
        
        # Update the detector level
        self.update_level(raw_value, time_constant)
        
        # Threshold with hysteresis
        if self.level > self.threshold:
            self.state = True
        elif self.level < 1 - self.threshold:
            self.state = False
            
        return self.state
    
    def reset(self, level=0.0, state=False):
        """
        Reset the detector to initial values.
        
        Args:
            level (float): Initial level value
            state (bool): Initial state value
        """
        super().reset(level)
        self.state = state


class SideEstimator(SensorEstimator):
    """
    Estimator for determining which side of the road the robot is on.
    Estimates whether the robot is in the center, on the left, or on the right side.
    """
    # Define constants for the three possible states
    LEFT = "LEFT"
    CENTER = "CENTER"
    RIGHT = "RIGHT"
    
    def __init__(self, initial_level=0.0, initial_state=CENTER, threshold=0.63):
        """
        Initialize the road-side estimator.
        
        Args:
            initial_level (float): Initial level for the running average (-1.0 to 1.0)
            initial_state (str): Initial state of the estimator (LEFT, CENTER, or RIGHT)
            threshold (float): Threshold for state changes (default 0.63, ~one time constant)
        """
        super().__init__(initial_level, threshold)
        self.state = initial_state
    
    def update(self, ir_readings, time_constant=0.5):
        """
        Update the road-side estimator with new IR readings.
        
        Args:
            ir_readings (tuple): Tuple of (left, middle, right) IR sensor readings
            time_constant (float): Time constant for the estimator (in seconds)
            
        Returns:
            str: Current state of the estimator (LEFT, CENTER, or RIGHT)
        """
        # Unpack the IR readings
        left, middle, right = ir_readings
        
        # Calculate raw value: 
        # 0.0 for centered
        # -0.5/-1.0 for slightly/significantly left
        # 0.5/1.0 for slightly/significantly right
        if middle == 1:
            if left == 1 and right == 0:
                raw_value = 0.5  # Slightly right 
            elif left == 0 and right == 1:
                raw_value = -0.5   # Slightly left
            elif left == 0 and right == 0:
                raw_value = 0.0   # Centered
            else:  # All sensors are 1 (possibly at intersection)
                raw_value = self.level  # Maintain current level
        else:  # Middle sensor is off the line
            if left == 1 and right == 0:
                raw_value = 1.0  # Significantly right 
            elif left == 0 and right == 1:
                raw_value = -1.0   # Significantly left 
            else:  # Both left and right are the same
                raw_value = self.level  # Maintain current level
        
        # Update the estimator level
        self.update_level(raw_value, time_constant)
        
        # Threshold with hysteresis for three states
        if self.level < -self.threshold:
            self.state = self.LEFT
        elif self.level > self.threshold:
            self.state = self.RIGHT
        elif abs(self.level) < (1 - self.threshold):
            self.state = self.CENTER
            
        return self.state
    
    def reset(self, level=0.0, state=CENTER):
        """
        Reset the estimator to initial values.
        
        Args:
            level (float): Initial level value
            state (str): Initial state value
        """
        super().reset(level)
        self.state = state


class EndOfStreetEstimator(SensorEstimator):
    """
    Detector for identifying the end of a street.
    """
    
    def __init__(self, initial_level=0.0, initial_state=False, threshold=0.63):
        """
        Initialize the end-of-street detector.
        
        Args:
            initial_level (float): Initial level for the running average (0.0-1.0)
            initial_state (bool): Initial state of the detector
            threshold (float): Threshold for state changes (default 0.63, ~one time constant)
        """
        super().__init__(initial_level, threshold)
        self.state = initial_state
        # Keep track of previous side estimate to help determine if robot is off-center
        self.prev_side = None
    
    def update(self, ir_readings, side_state, time_constant=1.0):
        """
        Update the end-of-street detector with new IR readings and side state.
        
        Args:
            ir_readings (tuple): Tuple of (left, middle, right) IR sensor readings
            side_state (str): Current state from SideEstimator (LEFT, CENTER, RIGHT)
            time_constant (float): Time constant for the detector (in seconds)
            
        Returns:
            bool: True if end of street is detected, False otherwise
        """
        # Unpack the IR readings
        left, middle, right = ir_readings
        
        # End of street is indicated by:
        # 1. No line detection on any sensor (all 0s)
        # 2. Robot is centered (not pushed off to the side)
        # 3. This condition persists for a sufficient distance
        
        # Calculate raw value:
        # 1.0 if we think it's the end of street, 0.0 otherwise
        if all(reading == 0 for reading in ir_readings):
            # All sensors read 0, but we need to check if we're centered
            if side_state == SideEstimator.CENTER:
                # Robot is centered with no line - likely end of street
                raw_value = 1.0
            else:
                # Robot is off-center - likely pushed off the side
                raw_value = 0.0
        else:
            # At least one sensor sees the line - not end of street
            raw_value = 0.0
        
        # Update the detector level
        self.update_level(raw_value, time_constant)
        
        # Store the current side state for next update
        self.prev_side = side_state
        
        # Threshold with hysteresis
        if self.level > self.threshold:
            self.state = True
        elif self.level < 1 - self.threshold:
            self.state = False
            
        return self.state
    
    def reset(self, level=0.0, state=False):
        """
        Reset the detector to initial values.
        
        Args:
            level (float): Initial level value
            state (bool): Initial state value
        """
        super().reset(level)
        self.state = state
        self.prev_side = None


class NextStreetDetector(SensorEstimator):
    """
    Detector for identifying when the robot has found the next street during a turn.
    Uses hysteresis to avoid false triggers from roadblocks or "false streets".
    
    Two scenarios:
    1. Starting with sensor on a line: Need to see off-road then on-road again
    2. Starting with sensor not on a line: Just need to see on-road
    """
    
    def __init__(self, initial_level=0.0, threshold=0.63):
        """
        Initialize the next-street detector.
        
        Args:
            initial_level (float): Initial level for the running average (0.0-1.0)
            threshold (float): Threshold for state changes (default 0.63, ~one time constant)
        """
        super().__init__(initial_level, threshold)
        self.initial_reading = None
        self.looking_for_off_road = False
        self.looking_for_on_road = False
        self.found_next_street = False
        self.state = False
    
    def update(self, ir_readings, time_constant=0.5):
        """
        Update the next-street detector with new IR readings.
        
        Args:
            ir_readings (tuple): Tuple of (left, middle, right) IR sensor readings
            time_constant (float): Time constant for the detector (in seconds)
            
        Returns:
            bool: True if the next street is detected, False otherwise
        """
        # We'll focus on the middle sensor only
        middle = ir_readings[1]
        
        # Record the initial reading on the first call and set up initial state
        if self.initial_reading is None:
            self.initial_reading = middle
            print(f"Turn started with middle sensor {'ON' if middle == 1 else 'OFF'} the line")
            
            # Set up the appropriate sequence based on initial reading
            if self.initial_reading == 1:
                # Start by looking for off-road
                self.looking_for_off_road = True
                self.looking_for_on_road = False
            else:
                # Start by looking for on-road
                self.looking_for_off_road = False
                self.looking_for_on_road = True
        
        # Calculate raw value based on what we're looking for
        raw_value = 0.0
        
        if self.looking_for_off_road:
            # Looking for middle sensor to be off the line (raw value 1.0 when off line)
            raw_value = 0.0 if middle == 1 else 1.0
            
            # Check if we've crossed the threshold to consider "off road"
            if self.level > self.threshold and not self.looking_for_on_road:
                # Transition to looking for on-road
                self.looking_for_off_road = False
                self.looking_for_on_road = True
                self.level = 0.0  # Reset level for next phase
                print("Turning: Off the current street")
                
        elif self.looking_for_on_road:
            # Looking for middle sensor to be on the line (raw value 1.0 when on line)
            raw_value = 1.0 if middle == 1 else 0.0
            
            # Check if we've crossed the threshold to consider "on road"
            if self.level > self.threshold and not self.found_next_street:
                # We've found the next street
                self.looking_for_on_road = False
                self.found_next_street = True
                print("Turning: Found the next street!")
        
        # Update the detector level
        self.update_level(raw_value, time_constant)
        
        # Threshold with hysteresis for final state
        if self.found_next_street:
            self.state = True
        
        return self.state
    
    def reset(self):
        """
        Reset the detector to initial values.
        """
        super().reset(0.0)
        self.initial_reading = None
        self.looking_for_off_road = False
        self.looking_for_on_road = False
        self.found_next_street = False
        self.state = False
