import pigpio
import traceback
import time

class Motor():
    def __init__(self, pin_a: int, pin_b: int, io: pigpio.pi, pwm_frequency: int):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.io = io
        self.pwm_frequency = pwm_frequency

        self.io.set_mode(self.pin_a, pigpio.OUTPUT)
        self.io.set_mode(self.pin_b, pigpio.OUTPUT)

        self.io.set_PWM_range(self.pin_a, 255)
        self.io.set_PWM_range(self.pin_b, 255)

        self.io.set_PWM_frequency(self.pin_a, self.pwm_frequency)
        self.io.set_PWM_frequency(self.pin_b, self.pwm_frequency)

        self.io.set_PWM_dutycycle(self.pin_a, 0)
        self.io.set_PWM_dutycycle(self.pin_b, 0)

    def stop(self):
        self.setlevel(0)

    def setlevel(self, level):
        try:

            if level < 0:
                self.io.set_PWM_dutycycle(self.pin_a, 0)
                self.io.set_PWM_dutycycle(self.pin_b, int(255 * abs(level)))
            else:
                self.io.set_PWM_dutycycle(self.pin_a, int(255 * abs(level)))
                self.io.set_PWM_dutycycle(self.pin_b, 0)

        except BaseException as e:
            print("Ending due to exception: %s" % repr(e))
            traceback.print_exc()


if __name__ == "__main__":

    PIN_MOTOR1_LEGA = 8
    PIN_MOTOR1_LEGB = 7
    PIN_MOTOR2_LEGA = 5
    PIN_MOTOR2_LEGB = 6

    io = pigpio.pi()
    motor1 = Motor(PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, io, 1000)
    motor2 = Motor(PIN_MOTOR2_LEGB, PIN_MOTOR2_LEGA, io, 1000)

    # 4.4
    def meter_drive(level, sleep_time):
        motor1.setlevel(level)
        motor2.setlevel(level)
        time.sleep(sleep_time)
        motor1.stop()
        motor2.stop()

    # meter_drive(1, 2.2)

    # 4.5
    def right_turn(level, sleep_time):
        motor1.setlevel(level)
        motor2.setlevel(-1 * level)
        time.sleep(sleep_time)
        motor1.stop()
        motor2.stop()

    # right_turn(1, 0.405)

    # 4.6
    def square_drive():
        for i in range(4):
            meter_drive(1, 2.4)
            time.sleep(1.0)
            right_turn(0.8, .59)
            time.sleep(1.0)

    # meter_drive(1, 2.45)
    # right_turn(0.8, 0.59)

    square_drive()

    # Clear the PINs (commands).
    io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 0)
    io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 0)
    io.set_PWM_dutycycle(PIN_MOTOR2_LEGA, 0)
    io.set_PWM_dutycycle(PIN_MOTOR2_LEGB, 0)
    
    io.stop()