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
import numpy as np

# pwm values for each motor when the wall is to the right of the robot
l_pwm_vals = np.array([
            0, 
            DriveSystem.DRIVE_STYLES[STEER_L][0],
            DriveSystem.DRIVE_STYLES[VEER_L][0],
            DriveSystem.DRIVE_STYLES[STRAIGHT][0],
            DriveSystem.DRIVE_STYLES[VEER_R][0],
            DriveSystem.DRIVE_STYLES[STEER_R][0],
            0
        ])
r_pwm_vals = np.array([
            0, 
            DriveSystem.DRIVE_STYLES[STEER_L][1],
            DriveSystem.DRIVE_STYLES[VEER_L][1],
            DriveSystem.DRIVE_STYLES[STRAIGHT][1],
            DriveSystem.DRIVE_STYLES[VEER_R][1],
            DriveSystem.DRIVE_STYLES[STEER_R][1],
            0
        ])

def herding_behavior(drive_system, sensors):
    """
    Controls robot to navigate by reacting to sensor readings.

    Args:
        drive_system: Manages robot's movement.
        sensors: Provides distance readings (left, middle, right).
    """
    sensors.trigger()
    last_trigger_time = time.time()
    while True:
        now = time.time()
        if now - last_trigger_time > 0.05:
            sensors.trigger()
            last_trigger_time = now
        
        left, middle, right = sensors.read()
        
        if None in (left, middle, right):
            print("Waiting for valid sensor readings...")
            continue
        if left > MAX_THRESHOLD and middle > MAX_THRESHOLD and right > MAX_THRESHOLD:
            # GO STRAIGHT
            drive_system.drive(STRAIGHT)
        elif left > MAX_THRESHOLD and middle > MAX_THRESHOLD and right <= MAX_THRESHOLD:
            # GO FORWARD WHILE MOVING SLIGHTLY LEFT
            drive_system.drive(TURN_L)
        elif left <= MAX_THRESHOLD and middle > MAX_THRESHOLD and right > MAX_THRESHOLD:
            # GO FORWARD while MOVING SLIGHTLY RIGHT
            drive_system.drive(TURN_R)
        elif left <= MAX_THRESHOLD and middle > MAX_THRESHOLD and right <= MAX_THRESHOLD:
            # GO STRAIGHT
            drive_system.drive(STRAIGHT)
        elif left > MAX_THRESHOLD and (MIN_THRESHOLD <= middle <= MAX_THRESHOLD) and right > MAX_THRESHOLD:
            # no forward motion so we STOP
            drive_system.stop()
        elif left > MAX_THRESHOLD and (MIN_THRESHOLD <= middle <= MAX_THRESHOLD) and right <= MAX_THRESHOLD:
            # STOP and ROTATE LEFT
            drive_system.drive(SPIN_L)
        elif left <= MAX_THRESHOLD and (MIN_THRESHOLD <= middle <= MAX_THRESHOLD) and right > MAX_THRESHOLD:
            # STOP and ROTATE RIGHT
            drive_system.drive(SPIN_R)
        elif left <= MAX_THRESHOLD and (MIN_THRESHOLD <= middle <= MAX_THRESHOLD) and right <= MAX_THRESHOLD:
            # no forward motion and we left and right are below threshold so STOP
            drive_system.stop()
        elif left > MAX_THRESHOLD and middle < MIN_THRESHOLD and right > MAX_THRESHOLD:
            # GO BACKWARDS
            drive_system.drive(STRAIGHT, backwards=True)
        elif left > MAX_THRESHOLD and middle < MIN_THRESHOLD and right <= MAX_THRESHOLD:
            # GO BACKWARDS and STEER LEFT
            drive_system.drive(TURN_R, backwards=True)
        elif left <= MAX_THRESHOLD and middle < MIN_THRESHOLD and right > MAX_THRESHOLD:
            # go BACKWARDS and TURN RIGHT
            drive_system.drive(TURN_L, backwards=True)
        elif left <= MAX_THRESHOLD and middle < MIN_THRESHOLD and right <= MAX_THRESHOLD:
            # GO BACKWARDS
            drive_system.drive(STRAIGHT, backwards=True)

def wall_following_behavior(drive_system, sensors):
    """
    Enables robot to follow a wall using error-based control.

    Args:
        drive_system: Manages robot's movement.
        sensors: Provides distance readings (left, middle, right).
    """
    sensors.trigger()
    last_trigger_time = time.time()
    direction =  (
                input(
                    "Enter direction of wall (l=left, r=right): "
                )
                .strip()
                .lower()
            )
    while True:
        now = time.time()
        if now - last_trigger_time > 0.05:
            sensors.trigger()
            last_trigger_time = now

        left, middle, right = sensors.read()
        if None in (left, middle, right):
            print("Waiting for valid sensor readings...")
            continue

        # Check for obstacles in front (middle sensor)
        if middle < MIN_THRESHOLD:
            print("Obstacle detected in front! Stopping...")
            drive_system.stop()
            continue

        if direction == 'l':
            error = left - NOMINIAL_DISTANCE
            print(f"Left={left:.2f}, Error={error:.2f}")

            # Error bands for left wall following:
            # |error| > 0.1m: Stop (safety)
            # 0.07m < error <= 0.1m: Steer left
            # 0.03m < error <= 0.07m: Veer left
            # -0.03m <= error <= 0.03m: Go straight
            # -0.07m <= error < -0.03m: Veer right
            # -0.1m <= error < -0.07m: Steer right
            # error < -0.1m: Stop (safety)
            if error > 0.1:
                drive_system.stop()
            elif error > 0.07:
                drive_system.drive(STEER_L)
            elif error > 0.03:
                drive_system.drive(VEER_L)
            elif error < -0.1:
                drive_system.stop()
            elif error < -0.07:
                drive_system.drive(STEER_R)
            elif error < -0.03:
                drive_system.drive(VEER_R)
            else:
                drive_system.drive(STRAIGHT)
        else:
            error = right - NOMINIAL_DISTANCE
            print(f"Right={right:.2f}, Error={error:.2f}")

            # Error bands for right wall following:
            # |error| > 0.1m: Stop (safety)
            # 0.07m < error <= 0.1m: Steer right
            # 0.03m < error <= 0.07m: Veer right
            # -0.03m <= error <= 0.03m: Go straight
            # -0.07m <= error < -0.03m: Veer left
            # -0.1m <= error < -0.07m: Steer left
            # error < -0.1m: Stop (safety)
            if error > 0.1:
                drive_system.stop()
            elif error > 0.07:
                drive_system.drive(STEER_R)
            elif error > 0.03:
                drive_system.drive(VEER_R)
            elif error < -0.1:
                drive_system.stop()
            elif error < -0.07:
                drive_system.drive(STEER_L)
            elif error < -0.03:
                drive_system.drive(VEER_L)
            else:
                drive_system.drive(STRAIGHT)
            
def fit(direction):
    """
    Calculates linear regression coefficients for motor PWM.

    Args:
        direction (str): 'l' for left wall, 'r' for right wall.

    Returns:
        tuple: (c_l, k_l, c_r, k_r) - intercepts and slopes for left/right motors.
    """
    error_bands = np.array([-0.1, -0.07, -0.03, 0, 0.03, 0.07, 0.1])
    # Use local copies to avoid modifying global state or causing UnboundLocalError
    l_vals = l_pwm_vals
    r_vals = r_pwm_vals

    if direction != 'r':
        l_vals = l_vals[::-1]
        r_vals = r_vals[::-1]

    k_l, c_l = np.polyfit(error_bands, l_vals, 1)
    k_r, c_r = np.polyfit(error_bands, r_vals, 1)
    print(f'c_l={c_l}, k_l={k_l}, c_r={c_r}, k_r={k_r}')
    return c_l, k_l, c_r, k_r


def wall_following_behavior_new(drive_system, sensors):
    """
    Implements advanced wall-following using linear regression for PWM control.

    Args:
        drive_system: DriveSystem object.
        sensors: IR sensors.
    """
    sensors.trigger()
    last_trigger_time = time.time()
    direction =  (
                input(
                    "Enter direction of wall (l=left, r=right): "
                )
                .strip()
                .lower()
            )
    while True:
        now = time.time()
        if now - last_trigger_time > 0.05:
            sensors.trigger()
            last_trigger_time = now

        left, middle, right = sensors.read()
        if None in (left, middle, right):
            print("Waiting for valid sensor readings...")
            continue

        # Check for obstacles in front (middle sensor)
        if middle < MIN_THRESHOLD:
            print("Obstacle detected in front! Stopping...")
            drive_system.stop()
            continue
        error = left - NOMINIAL_DISTANCE if direction == 'l' else right - NOMINIAL_DISTANCE
        print(f"Error={error:.2f}")
        c_l, k_l, c_r, k_r = fit(direction)
        pwm_l, pwm_r = PWM_CONST * (c_l + error * k_l), PWM_CONST * (c_r + error * k_r)
        drive_system.pwm(pwm_l, pwm_r)


if __name__ == "__main__":
    # Testing herding/wall following behavior
    io = pigpio.pi()

    # Create drive system and sensors
    sensors = ProximitySensor(io)
    ds = DriveSystem(io)
    try:
        wall_following_behavior_new(ds, sensors)
        # herding_behavior(ds, sensors)

    except BaseException as e:
        ds.stop()
        io.stop()
        print("Ending due to exception: %s" % repr(e))
        traceback.print_exc()