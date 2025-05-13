from config import (
    PIN_US_TRIGGER_L,
    PIN_US_TRIGGER_M,
    PIN_US_TRIGGER_R,
    PIN_US_ECHO_L,
    PIN_US_ECHO_M,
    PIN_US_ECHO_R
)
import pigpio
import time
microsec_10 = 0.000010
class Ultrasound:
    def __init__(self, io, pin_trig, pin_echo):
        self.io = io
        self.pin_trig = pin_trig
        self.pin_echo = pin_echo
        self.io.set_mode(pin_trig, pigpio.OUTPUT)
        self.io.set_mode(pin_echo, pigpio.INPUT)
        self.cbrise = io.callback(pin_echo, pigpio.RISING_EDGE, self.rising)
        self.cbfall = io.callback(pin_echo, pigpio.FALLING_EDGE, self.falling)
    
    def trigger(self):
        self.io.write(self.pin_trig, 1)
        t0 = time.time()
        curr = time.time()
        while curr - t0 <= microsec_10:
            curr = time.time()
        self.io.write(self.pin_trig, 0)
    
    def rising(self, pin, level, tick )