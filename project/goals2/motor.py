import pigpio
import traceback
import time

class Motor():
    """
    Motor controller using PWM through GPIO pins.
    """

    def __init__(self, pin_a: int, pin_b: int, io: pigpio.pi, pwm_frequency: int):
        """
        Initialize a new motor controller.
        
        Args:
            pin_a (int): GPIO pin number for motor leg A
            pin_b (int): GPIO pin number for motor leg B  
            io (pigpio.pi): pigpio interface instance
            pwm_frequency (int): PWM frequency in Hz
        """
        # Initialize pins, io, and pwm frequency
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.io = io
        self.pwm_frequency = pwm_frequency

        # Set pins as output
        self.io.set_mode(self.pin_a, pigpio.OUTPUT)
        self.io.set_mode(self.pin_b, pigpio.OUTPUT)

        # Prepare the PWM
        # The range gives the maximum value for 100% duty cycle, using integer commands (1 up to max)
        self.io.set_PWM_range(self.pin_a, 255)
        self.io.set_PWM_range(self.pin_b, 255)
        # Set the PWM frequency to 1000Hz
        self.io.set_PWM_frequency(self.pin_a, self.pwm_frequency)
        self.io.set_PWM_frequency(self.pin_b, self.pwm_frequency)

        # Clear pins
        self.stop()

    def stop(self):
        """Stop the motor by setting PWM level to 0."""
        self.setlevel(0)

    def setlevel(self, level):
        """
        Set the motor speed and direction.
        
        Args:
            level (float): Motor power level from -1.0 to 1.0.
                         Negative values spin backwards, positive forwards.
        """
        if level < 0:
            self.io.set_PWM_dutycycle(self.pin_a, 0)
            self.io.set_PWM_dutycycle(self.pin_b, int(255 * abs(level)))
        else:
            self.io.set_PWM_dutycycle(self.pin_a, int(255 * abs(level)))
            self.io.set_PWM_dutycycle(self.pin_b, 0)


if __name__ == "__main__":
    try:

        PIN_MOTOR1_LEGA = 8
        PIN_MOTOR1_LEGB = 7
        PIN_MOTOR2_LEGA = 5
        PIN_MOTOR2_LEGB = 6

        io = pigpio.pi()
        motor1 = Motor(PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, io, 1000)
        motor2 = Motor(PIN_MOTOR2_LEGB, PIN_MOTOR2_LEGA, io, 1000)

        # 4.4
        def meter_drive(level1, level2, sleep_time):
            motor1.setlevel(level1)
            motor2.setlevel(level2)
            time.sleep(sleep_time)
            motor1.stop()
            motor2.stop()

        # 4.5
        def right_turn(level, sleep_time):
            motor1.setlevel(level)
            motor2.setlevel(-1 * level)
            time.sleep(sleep_time)
            motor1.stop()
            motor2.stop()

        # 4.6
        def square_drive():
            for i in range(4):
                meter_drive(1, 2.4)
                time.sleep(1.0)
                right_turn(0.8, .59)
                time.sleep(1.0)

        # meter_drive(0.9, 0.878, 2.6)
        # right_veer = 1, 0.9, 5
        
        # square_drive()

        # Clear the PINs (commands).
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 0)
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 0)
        io.set_PWM_dutycycle(PIN_MOTOR2_LEGA, 0)
        io.set_PWM_dutycycle(PIN_MOTOR2_LEGB, 0)

        io.stop()

    except BaseException as e:
        print("Ending due to exception: %s" % repr(e))
        traceback.print_exc()
        