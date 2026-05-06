from stepper_module import StepperMotor
import RPi.GPIO as GPIO
import time

step_delay = 0.01
open_pins = [31, 33, 35, 37]
open_stepper = StepperMotor(open_pins)

flip_pins = [32, 36, 38, 40]
flip_stepper = StepperMotor(flip_pins)

try:

    open_stepper.move(-2400, step_delay)

except KeyboardInterrupt:
    print("\nShutting down motors...")
finally:
    # Clean up all GPIO pins
    GPIO.cleanup()