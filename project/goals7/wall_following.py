import pigpio
from drive_system import DriveSystem

from proximitysensor import ProximitySensor
import traceback
import time
from config import (
    STRAIGHT,
    VEER_L,
    STEER_L,
    VEER_R,
    STEER_R,
    TURN_L,
    HOOK_L,
    TURN_R,
    HOOK_R,
    SPIN_L,
    SPIN_R,
    MAX_THRESHOLD,
    MIN_THRESHOLD,
    NOMINIAL_DISTANCE,
    PWM_CONST
)
from ultrasound_behaviors import l_pwm_vals, r_pwm_vals, fit
import numpy as np
from ui_wall_following import runui, SharedData
import threading

def main():
    io = pigpio.pi()

    proximity = ProximitySensor(io)
    drive = DriveSystem(io)

    shared = SharedData()

    ui_thread = threading.Thread(name="UIThread", target=runui, args=(shared, ))
    ui_thread.daemon = True
    ui_thread.start()

    runrobot(shared, drive, proximity)

    print("Cleaning up hardware...")
    drive.stop()
    proximity.shutdown()
    io.stop()
    print("Cleanup complete.")

def runrobot(shared, drive, proximity):

    try:
        # Keep the brain running unless told to stop
        running = True
        
        while running:
            last_trigger_time = time.time()

            action = ""

            # Grab access to the shared data
            if shared.acquire():
                # If told to quit, DO NOT BREAK but clear the running flag
                if shared.mode == -1:
                    running = False
                
                elif shared.mode == 0:
                    action = "stop"

                elif shared.mode == 1:
                    action = "discrete"

                elif shared.mode == 2:
                    action = "continuous"

                direction = shared.wall_following_side
                
                shared.release()

            if action == "stop":
                drive.stop()

            elif action == "discrete":
                now = time.time()
                if now - last_trigger_time > 0.05:
                    last_trigger_time = now
                left, middle, right = proximity.read()
                if None in (left, middle, right):
                    continue
                if middle < MIN_THRESHOLD:
                    drive.stop()
                    continue
                if direction == 'l':
                    error = left - NOMINIAL_DISTANCE
                    if error > 0.1:
                        drive.stop()
                    elif error > 0.07:
                        drive.drive(STEER_L)
                    elif error > 0.03:
                        drive.drive(VEER_L)
                    elif error < -0.1:
                        drive.stop()
                    elif error < -0.07:
                        drive.drive(STEER_R)
                    elif error < -0.03:
                        drive.drive(VEER_R)
                    else:
                        drive.drive(STRAIGHT)
                else:
                    error = right - NOMINIAL_DISTANCE
                    if error > 0.1:
                        drive.stop()
                    elif error > 0.07:
                        drive.drive(STEER_R)
                    elif error > 0.03:
                        drive.drive(VEER_R)
                    elif error < -0.1:
                        drive.stop()
                    elif error < -0.07:
                        drive.drive(STEER_L)
                    elif error < -0.03:
                        drive.drive(VEER_L)
                    else:
                        drive.drive(STRAIGHT)

            elif action == "continuous":
                now = time.time()
                if now - last_trigger_time > 0.05:
                    last_trigger_time = now
                left, middle, right = proximity.read()
                if None in (left, middle, right):
                    continue
                if middle < MIN_THRESHOLD:
                    drive.stop()
                    continue
                error = left - NOMINIAL_DISTANCE if direction == 'l' else right - NOMINIAL_DISTANCE
                c_l, k_l, c_r, k_r = fit(direction)

                pwm_l, pwm_r = PWM_CONST * (c_l + error * k_l), PWM_CONST * (c_r + error * k_r)
                try:
                    drive.pwm(pwm_l, pwm_r)
                except:
                    drive.pwm(0, 0)

    except BaseException as ex:
        print("\nEnding Run-Robot due to exception: %s" % repr(ex))

if __name__ == "__main__":
    main()
    