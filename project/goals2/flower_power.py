from drive_system import DriveSystem
from motor import Motor

import pigpio
import traceback
import time

#####################
# Flower Power Test #
#####################

PIN_MOTOR1_LEGA = 8
PIN_MOTOR1_LEGB = 7
PIN_MOTOR2_LEGA = 5
PIN_MOTOR2_LEGB = 6

io = pigpio.pi()
motor1 = Motor(PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, io, 1000)
motor2 = Motor(PIN_MOTOR2_LEGB, PIN_MOTOR2_LEGA, io, 1000)

ds = DriveSystem(io, motor1, motor2) 


for style in ["STRAIGHT", "VEER_L", "STEER_L", "TURN_L", "HOOK_L", "SPIN_L", "VEER_R", "STEER_R", "TURN_R", "HOoK_R", "SPIN_R"]:
    ds.drive(style)
    time.sleep(4)
    ds.stop()
    input("hit return")

io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 0)
io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 0)
io.set_PWM_dutycycle(PIN_MOTOR2_LEGA, 0)
io.set_PWM_dutycycle(PIN_MOTOR2_LEGB, 0)

io.stop()
