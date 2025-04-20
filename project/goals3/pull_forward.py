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
    
    def pull_forward(self, time_constant=0.5):
        t_0 = self.tlast
        while True:
            t_now = time.time()
            
    
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
                raw_value = -0.5  # Slightly left
            elif left == 0 and right == 1:
                raw_value = 0.5   # Slightly right
            elif left == 0 and right == 0:
                raw_value = 0.0   # Centered
            else:  # All sensors are 1 (possibly at intersection)
                raw_value = self.level  # Maintain current level
        else:  # Middle sensor is off the line
            if left == 1 and right == 0:
                raw_value = -1.0  # Significantly left
            elif left == 0 and right == 1:
                raw_value = 1.0   # Significantly right
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
