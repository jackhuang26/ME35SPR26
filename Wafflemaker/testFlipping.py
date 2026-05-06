from servo_module import Servo
from dc_motor_module import DCMotor
from stepper_heat_module import StepperMotor
import RPi.GPIO as GPIO
import time
import threading


step_delay = 0.01
open_pins = [31, 33, 35, 37]
open_stepper = StepperMotor(open_pins, power_level=100)

flip_pins = [32, 36, 38, 40]
flip_stepper = StepperMotor(flip_pins)

cook_time = 120  # Cook for 180 seconds or 3 minutes?

try:
    open_stepper.move(-1200, step_delay)
    open_stepper.stop()

    time.sleep(5)

    open_stepper.move(1200, step_delay)
    open_stepper.stop()

    # flip_stepper.move(2600, step_delay)
    # flip_stepper.stop()

    print("Finished")


except KeyboardInterrupt:
    print("\nShutting down motors...")
finally:
    # Clean up all GPIO pins
    GPIO.cleanup()