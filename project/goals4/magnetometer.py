import pigpio
import traceback
import math
from config import (
    PIN_MAG_LATCH,
    PIN_MAG_ADDRESS,
    PIN_MAG_READY,
    PIN_MAG_PIN0,
    PIN_MAG_PIN1,
    PIN_MAG_PIN2,
    PIN_MAG_PIN3,
    PIN_MAG_PIN4,
    PIN_MAG_PIN5,
    PIN_MAG_PIN6,
    PIN_MAG_PIN7,
    MIN_0,
    MAX_0,
    MIN_1,
    MAX_1,
)

PINS = {
    PIN_MAG_PIN0: 1,
    PIN_MAG_PIN1: 2,
    PIN_MAG_PIN2: 4,
    PIN_MAG_PIN3: 8,
    PIN_MAG_PIN4: 16,
    PIN_MAG_PIN5: 32,
    PIN_MAG_PIN6: 64,
    PIN_MAG_PIN7: 128,
}

SCALE = 180 / math.pi


class ADC:
    """
    Analog-to-Digital Converter interface for the magnetometer.
    
    This class provides methods to read raw ADC values from the magnetometer
    and convert them to angle measurements. It handles the low-level communication
    with the hardware through the pigpio interface.
    """
    
    def __init__(self, io):
        """
        Initialize the ADC interface with the pigpio instance.
        
        Sets up all the necessary GPIO pins for communication with the magnetometer.
        
        Args:
            io (pigpio.pi): pigpio interface instance for hardware communication
        """
        self.io = io
        self.io.set_mode(PIN_MAG_LATCH, pigpio.OUTPUT)
        self.io.set_mode(PIN_MAG_ADDRESS, pigpio.OUTPUT)
        self.io.set_mode(PIN_MAG_READY, pigpio.INPUT)
        self.io.set_mode(PIN_MAG_PIN0, pigpio.INPUT)
        self.io.set_mode(PIN_MAG_PIN1, pigpio.INPUT)
        self.io.set_mode(PIN_MAG_PIN2, pigpio.INPUT)
        self.io.set_mode(PIN_MAG_PIN3, pigpio.INPUT)
        self.io.set_mode(PIN_MAG_PIN4, pigpio.INPUT)
        self.io.set_mode(PIN_MAG_PIN5, pigpio.INPUT)
        self.io.set_mode(PIN_MAG_PIN6, pigpio.INPUT)
        self.io.set_mode(PIN_MAG_PIN7, pigpio.INPUT)

    def readadc(self, address):
        """
        Read a raw ADC value from the specified address.
        
        This method handles the low-level communication protocol with the magnetometer,
        including setting the address, toggling the latch, and reading the data pins.
        
        Args:
            address (int): The address to read from (0 or 1)
            
        Returns:
            int: The raw ADC value read from the specified address
        """
        output = 0
        self.io.write(PIN_MAG_LATCH, pigpio.LOW)
        self.io.write(PIN_MAG_ADDRESS, address)
        self.io.write(PIN_MAG_LATCH, pigpio.HIGH)
        self.io.write(PIN_MAG_LATCH, pigpio.LOW)
        self.io.write(PIN_MAG_LATCH, pigpio.HIGH)
        if self.io.read(PIN_MAG_READY):
            output += sum(PINS[pin] for pin in PINS if self.io.read(pin))
        return output

    def scale(self, min_val, max_val, curr_val):
        """
        Scale a value from its current range to the range [-1, 1].
        
        Args:
            min_val (float): Minimum value of the current range
            max_val (float): Maximum value of the current range
            curr_val (float): Current value to scale
            
        Returns:
            float: Scaled value in the range [-1, 1]
        """
        return 2 * ((curr_val - min_val) / (max_val - min_val)) - 1

    def readangle(self):
        """
        Read and calculate the current angle from the magnetometer.
        
        This method reads raw values from both ADC channels, scales them to the
        range [-1, 1], and calculates the angle using the arctangent function.
        
        Returns:
            float: The current angle in degrees
        """
        ad_0 = self.readadc(0)
        ad_1 = self.readadc(1)
        scaled_ad0, scaled_ad1 = (
            self.scale(MIN_0, MAX_0, ad_0),
            self.scale(MIN_1, MAX_1, ad_1),
        )
        # print(f"Raw ADC0: {ad_0}, Scaled ADC0: {scaled_ad0}")
        # print(f"Raw ADC1: {ad_1}, Scaled ADC1: {scaled_ad1}")
        return math.atan2(scaled_ad0, scaled_ad1) * SCALE


def test1():
    """
    Test function to calibrate the magnetometer.
    
    This function continuously reads raw ADC values from both channels
    and tracks the minimum and maximum values to help determine the
    calibration constants MIN_0, MAX_0, MIN_1, and MAX_1.
    """
    try:
        io = pigpio.pi()
        adc = ADC(io)
        min_0, max_0 = 255, 0
        min_1, max_1 = 255, 0
        while True:
            try:
                adc_0 = adc.readadc(0)
                adc_1 = adc.readadc(1)
                min_0, max_0 = min(min_0, adc_0), max(max_0, adc_0)
                min_1, max_1 = min(min_1, adc_1), max(max_1, adc_1)
                print(f"reading address 0: {adc_0}")
                print(f"reading address 1: {adc_1}")
            except KeyboardInterrupt:
                print(
                    f"min/max of addr 0: {(min_0, max_0)}, min/max of addr 1: {(min_1, max_1)}"
                )
                print("Quitting and shutting off io...")
                io.stop()
        io.stop()
    except BaseException as e:
        print("Ending due to exception: %s" % repr(e))
        traceback.print_exc()


def test3():
    """
    Test function to verify the angle reading functionality.
    
    This function continuously reads and displays the current angle
    from the magnetometer, allowing for verification of the angle
    calculation and calibration.
    """
    try:
        io = pigpio.pi()
        adc = ADC(io)
        while True:
            try:
                adc.readangle()
                print(f"Robot angle {adc.readangle()}")
            except KeyboardInterrupt:
                print("Quitting and shutting off io...")
                io.stop()
        io.stop()
    except BaseException as e:
        print("Ending due to exception: %s" % repr(e))
        traceback.print_exc()


if __name__ == "__main__":
    test3()
