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
microsec_10 = 1e-5
speed_of_sound = 343 # m/s
micro_to_sec = 1e-6
sec_to_micro = 1e6


class Ultrasound:
    def __init__(self, io, pin_trig, pin_echo):
        self.io = io
        self.pin_trig = pin_trig
        self.pin_echo = pin_echo
        self.io.set_mode(pin_trig, pigpio.OUTPUT)
        self.io.set_mode(pin_echo, pigpio.INPUT)
        self.cbrise = io.callback(pin_echo, pigpio.RISING_EDGE, self.rising)
        self.cbfall = io.callback(pin_echo, pigpio.FALLING_EDGE, self.falling)
        self.risetick = None
        self.distance = None
        self.time = None
    def trigger(self):
        self.io.write(self.pin_trig, pigpio.HIGH)
        time.sleep(microsec_10)
        self.io.write(self.pin_trig, pigpio.LOW)
    
    def rising(self, pin, level, ticks):
        self.risetick = ticks
    
    def falling(self, pin, level, ticks):
        deltatick = ticks - self.risetick
        if deltatick < 0:
            deltatick += 2**32
        self.time = deltatick * micro_to_sec / 2
        # print(deltatick * micro_to_sec / 2)
        self.distance = speed_of_sound / sec_to_micro * deltatick / 2

    def read(self):
        return self.distance if not None else print('No distance readings.')
    def read_time(self):
        return self.time if not None else print('No distance readings.')
    

class ProximitySensor:
    def __init__(self, io):
        self.io = io
        self.left_us = Ultrasound(self.io, PIN_US_TRIGGER_L, PIN_US_ECHO_L)
        self.middle_us = Ultrasound(self.io, PIN_US_TRIGGER_M, PIN_US_ECHO_M)
        self.right_us = Ultrasound(self.io, PIN_US_TRIGGER_R, PIN_US_ECHO_R)
    
    def trigger(self):
        self.left_us.trigger()
        self.middle_us.trigger()
        self.right_us.trigger()
        
    def read(self):
        return self.left_us.read(), self.middle_us.read(), self.right_us.read()
    def read_time(self):
        return self.left_us.read_time(), self.middle_us.read_time(), self.right_us.read_time()
    
def main():
    io = pigpio.pi()
    sensors = ProximitySensor(io)
    while True:
        # Trigger ultrasound and wait 50 sec
        sensors.trigger()
        time.sleep(0.050)
        # read/report
        distances = sensors.read()
        print("Distances = (%6.3fm, %6.3fm, %6.3fm)" % distances)

def test_5a():
    io = pigpio.pi()
    sensors = ProximitySensor(io)
    while True:
        # Trigger ultrasound and wait 50 sec
        sensors.trigger()
        time.sleep(0.050)
        # read/report
        times = sensors.read_time()
        print("Times = (%6.5fs, %6.5fs, %6.5fs)" % times)

def test_5b():
    io = pigpio.pi()
    sensors = ProximitySensor(io)
    while True:
        # Trigger ultrasound and wait 50 sec
        sensors.trigger()
        time.sleep(0.050)
        # read/report
        distances = sensors.read()
        print("Distances = (%6.3fm, %6.3fm, %6.3fm)" % distances)

def test_5c():
    io = pigpio.pi()
    sensors = ProximitySensor(io)
    min_r, max_r = float("inf"), float("-inf")
    while True:
        # Trigger ultrasound and wait 50 sec
        sensors.trigger()
        time.sleep(0.050)
        # read/report
        distances = sensors.read()
        min_r, max_r = min(min_r, distances[2]), max(max_r, distances[2])
        print(f'min_r: {min_r}, max_r: {max_r}')

if __name__ == "__main__":
    test_5b()
    