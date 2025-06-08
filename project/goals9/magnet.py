from config import PIN_MAGNET
import pigpio

class ElectroMagnet:
    def __init__(self, io):
        self.io = io
        self.io.set_mode(PIN_MAGNET, pigpio.OUTPUT)
    
    def on(self):
        self.io.write(PIN_MAGNET, pigpio.HIGH)
    
    def off(self):
        self.io.write(PIN_MAGNET, pigpio.LOW)