from servo_module import Servo
from dc_motor_module import DCMotor
from stepper_module import StepperMotor
import RPi.GPIO as GPIO
import time

spray_servo = Servo(18)
twist_servo = Servo(16)

dispense_motor = DCMotor(7, 11)

open_pins = [31, 33, 35, 37]
step_delay = 0.01
open_stepper = StepperMotor(open_pins)

try:

    open_stepper.move(-1200, step_delay)
    open_stepper.stop()

    spray_servo.set_angle(-30) 
    time.sleep(1)
    spray_servo.set_angle(30)
    time.sleep(1)

    twist_servo.set_angle(90)
    time.sleep(1)

    spray_servo.set_angle(-30) 
    time.sleep(1)
    spray_servo.set_angle(30)
    time.sleep(1)

    twist_servo.set_angle(0)
    time.sleep(1)

    dispense_motor.forward()
    time.sleep(3)
    dispense_motor.stop()
    time.sleep(2)
    dispense_motor.backward()
    time.sleep(3)
    dispense_motor.stop()

    open_stepper.move(1200, step_delay)
    open_stepper.stop()


except KeyboardInterrupt:
    print("\nShutting down motors...")
finally:
    # Clean up all GPIO pins
    GPIO.cleanup()