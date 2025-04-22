import pigpio
import sys
import traceback
from config import PIN_IR_LEFT, PIN_IR_MIDDLE, PIN_IR_RIGHT


class IR:
    """
    IR sensor class for each individual sensor.
    """

    # 3.3
    def __init__(self, io, pin_ir):
        """Initializes a IR object for each IR pin

        Args:
            io (pigpio.pi): pigpio interface instance
            pin_ir (int): GPIO pin number for designated IR
        """
        # initialization ...
        self.io = io
        self.pin = pin_ir
        self.io.set_mode(self.pin, pigpio.INPUT)

    def read(self):
        """Read the IR data

        Returns:
            int: 0 if the IR read white and 1 if it reads black
        """
        # read IR sensor data ...
        ir = self.io.read(self.pin)
        return ir


class LineSensor:
    """
    Class that groups all our sensors into one object. This will contain 3 IR
    sensors.
    """

    # 3.3
    def __init__(self, io, left, mid, right):
        """Initialization of a group of sensors object

        Args:
            io (pigpio.pi): pigpio interface instance
            left (int): GPIO pin number for left IR sensor
            mid (int): GPIO pin number for middle IR sensor
            right (int): GPIO pin number for right IR sensor
        """
        # initialization ...
        self.io = io
        self.left = IR(io, left)
        self.mid = IR(io, mid)
        self.right = IR(io, right)

    def read(self):
        """Read the input data of all three sensors

        Returns:
            tuple: input data of the left, middle, and right IR sensors
        """
        # read IR sensor data ...
        irl, irm, irr = self.left.read(), self.mid.read(), self.right.read()
        # print("IRs: L %d  M %d  R %d" % (irl, irm, irr))
        return (irl, irm, irr)


if __name__ == "__main__":
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")
    sensors = LineSensor(io, PIN_IR_LEFT, PIN_IR_MIDDLE, PIN_IR_RIGHT)
    try:
        while True:
            sensors.read()
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
