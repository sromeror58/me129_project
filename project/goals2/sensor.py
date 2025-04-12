import pigpio
import sys
import time
import traceback

# Define the IR pins.
PIN_IR_LEFT   = 14      # Default GPIO Channel for Left   IR Detector
PIN_IR_MIDDLE = 15      # Default GPIO Channel for Middle IR Detector
PIN_IR_RIGHT  = 18      # Default GPIO Channel for Right  IR Detector

class IR():
    # 3.3
   def __init__(self, io, pin_ir):
        # initialization ...
        self.io = io
        self.pin = pin_ir
        self.io.set_mode(self.pin, pigpio.INPUT)
   def read(self):
        # read IR sensor data ...
        ir = self.io.read(self.pin)
        return ir


class LineSensor():
    # 3.3
   def __init__(self, io, left, mid, right):
       # initialization ...
       self.io = io
       self.left = IR(io, left)
       self.mid = IR(io, mid)
       self.right = IR(io, right)   
   def read(self):
       # read IR sensor data ...
       irl, irm, irr = self.left.read(), self.mid.read(), self.right.read()
       print("IRs: L %d  M %d  R %d" % (irl, irm, irr))
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