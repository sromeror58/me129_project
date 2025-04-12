import pigpio
import sys
import time
import traceback


class IR():
   def __init__(self, io, pin_ir):
       # initialization ...
       self.io = io
       self.pin = pin_ir
       self.io.set_mode(self.pin, pigpio.INPUT)
   def read(self):
       # read IR sensor data ...
       try:
           while True:
               ir = self.io.read(self.pin)
               # printing data
               print(...)
       except BaseException as ex:
       # Report the error, then continue with the normal shutdown.
           print("Ending due to exception: %s" % repr(ex))
           traceback.print_exc()


class LineSensor():
   def __init__(self, io, left, mid, right):
       # initialization ...
       self.io = io
       self.left = IR(io, left)
       self.mid = IR(io, mid)
       self.right = IR(io, right)   
   def read(self):
       # read IR sensor data ...
       self.left.read()
       self.mid.read()
       self.right.read()