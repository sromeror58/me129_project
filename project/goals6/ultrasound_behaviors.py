import pigpio
from drive_system import DriveSystem
from proximitysensor import ProximitySensor
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
)

MAX_THRESHOLD = 0.2
MIN_THRESHOLD = 0.1


def heding_behavior(drive_system, sensors):
    sensors.trigger()
    last_trigger_time = time.time()
    
    while True:
        now = time.time()
        if last_trigger_time - now > 0.05:
            sensors.trigger()
            last_trigger_time = now
        
        left, middle, right = sensors.read()
        
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
            drive_system.drive(TURN_L, backwards=True)
        elif left <= MAX_THRESHOLD and middle < MIN_THRESHOLD and right > MAX_THRESHOLD:
            # go BACKWARDS and TURN RIGHT
            drive_system.drive(TURN_R, backwards=True)
        elif left <= MAX_THRESHOLD and middle < MIN_THRESHOLD and right <= MAX_THRESHOLD:
            # GO BACKWARDS
            drive_system.drive(STRAIGHT, backwards=True)

if __name__ == "__main__":
    # Testing herding behavior
    io = pigpio.pi()

    # Create drive system and sensors
    sensors = ProximitySensor(io)
    ds = DriveSystem(io)
    try:
        herding_behavior(ds, sensors)

    except BaseException as e:
        ds.stop()
        io.stop()
        print("Ending due to exception: %s" % repr(e))
        traceback.print_exc()
    
            
        