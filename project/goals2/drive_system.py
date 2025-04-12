import pigpio
import traceback
import time
from motor import Motor

class DriveValues():
    def __init__(self, level1: float, level2: float):
        self.level1 = level1
        self.level2 = level2

class DriveSystem():

    DRIVE_STYLES = {
        "STRAIGHT": (0.7537, 0.719),

        "VEER_L": (0.73, 0.77),
        "STEER_L": (0.62, 0.77),
        "TURN_L": (0.47, 0.79), 
        "HOOK_L": (0.0, 0.65),
        "SPIN_L": (-0.7, 0.65), # might be done

        "VEER_R": (0.77, 0.67),
        "STEER_R": (0.79, 0.60),
        "TURN_R": (0.82, 0.45),
        "HOOK_R": (0.64, 0),
        "SPIN_R": (0.75, -0.70), # need to polish
    }
    
    def __init__(self, io, motor1: Motor, motor2: Motor):
        self.io = io
        self.motor1 = motor1
        self.motor2 = motor2 
        self.drive_styles = {}
        for style, values in self.DRIVE_STYLES.items():
            self.drive_styles[style] = DriveValues(*values)

    def stop(self):
        self.motor1.stop()
        self.motor2.stop()

    def drive(self, style: str):
        if style not in self.drive_styles:
            raise ValueError(f"Invalid drive style: {style}")
        drive_val = self.drive_styles[style]
        self.motor1.setlevel(drive_val.level1)
        self.motor2.setlevel(drive_val.level2)

if __name__ == "__main__":
    # testing purposes for different drive modes
    try:
        PIN_MOTOR1_LEGA = 8
        PIN_MOTOR1_LEGB = 7
        PIN_MOTOR2_LEGA = 5
        PIN_MOTOR2_LEGB = 6

        io = pigpio.pi()
        motor1 = Motor(PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, io, 1000)
        motor2 = Motor(PIN_MOTOR2_LEGB, PIN_MOTOR2_LEGA, io, 1000)

        ds = DriveSystem(io, motor1, motor2) 
        ds.drive("SPIN_L")
        time.sleep(4)
        ds.stop()

        # Clear the PINs (commands).
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 0)
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 0)
        io.set_PWM_dutycycle(PIN_MOTOR2_LEGA, 0)
        io.set_PWM_dutycycle(PIN_MOTOR2_LEGB, 0)

        io.stop()

    except BaseException as e:
        print("Ending due to exception: %s" % repr(e))
        traceback.print_exc()
        

