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
    MAX_1
)

PINS = {
    PIN_MAG_PIN0: 1,
    PIN_MAG_PIN1: 2,
    PIN_MAG_PIN2: 4,
    PIN_MAG_PIN3: 8,
    PIN_MAG_PIN4: 16,
    PIN_MAG_PIN5: 32,
    PIN_MAG_PIN6: 64,
    PIN_MAG_PIN7: 128}

SCALE = 180 / math.pi

class ADC:
    def __init__(self, io):
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
        return 2*((curr_val - min_val)/(max_val - min_val)) - 1
        
    def readangle(self):
        ad_0 = self.readadc(0)
        ad_1 = self.readadc(1)
        scaled_ad0, scaled_ad1 = self.scale(MIN_0, MAX_0, ad_0), self.scale(MIN_1, MAX_1, ad_1)
        # print(f"Raw ADC0: {ad_0}, Scaled ADC0: {scaled_ad0}")
        # print(f"Raw ADC1: {ad_1}, Scaled ADC1: {scaled_ad1}")
        return math.atan2(scaled_ad0, scaled_ad1) * SCALE
        
def test1():
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
                print(f'reading address 0: {adc_0}')
                print(f'reading address 1: {adc_1}')
            except KeyboardInterrupt:
                print(f'min/max of addr 0: {(min_0, max_0)}, min/max of addr 1: {(min_1, max_1)}')
                print('Quitting and shutting off io...')
                io.stop()
        io.stop()
    except BaseException as e:
        print("Ending due to exception: %s" % repr(e))
        traceback.print_exc()

def test3():
    try:
        io = pigpio.pi()
        adc = ADC(io)
        while True:
            try:
                adc.readangle()
                print(f'Robot angle {adc.readangle()}')
            except KeyboardInterrupt:
                print('Quitting and shutting off io...')
                io.stop()
        io.stop()
    except BaseException as e:
        print("Ending due to exception: %s" % repr(e))
        traceback.print_exc()

if __name__ == "__main__":
    test3()
            
        