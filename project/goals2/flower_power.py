from drive_system import DriveSystem
from motor import Motor
from config import PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, PIN_MOTOR2_LEGA, PIN_MOTOR2_LEGB

import pigpio
import time

#####################
# Flower Power Test #
#####################

io = pigpio.pi()
motor1 = Motor(PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, io, 1000)
motor2 = Motor(PIN_MOTOR2_LEGB, PIN_MOTOR2_LEGA, io, 1000)

ds = DriveSystem(io, motor1, motor2)

for style in ds.DRIVE_STYLES:
    ds.drive(style)
    time.sleep(4)
    ds.stop()
    input("hit return")

io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 0)
io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 0)
io.set_PWM_dutycycle(PIN_MOTOR2_LEGA, 0)
io.set_PWM_dutycycle(PIN_MOTOR2_LEGB, 0)

io.stop()
