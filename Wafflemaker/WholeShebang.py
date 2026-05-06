from servo_module import Servo
from dc_motor_module import DCMotor
from stepper_heat_module import StepperMotor
import RPi.GPIO as GPIO
import time

spray_servo = Servo(18)
twist_servo = Servo(16)

dispense_motor = DCMotor(7, 11)

step_delay = 0.01
open_pins = [31, 33, 35, 37]
open_stepper = StepperMotor(open_pins, power_level=100)

flip_pins = [32, 36, 38, 40]
flip_stepper = StepperMotor(flip_pins)

cook_time = 120  # Cook for 180 seconds or 3 minutes?

try:

    # Open the waffle iron
    print("Opening waffle iron...")
    #open_stepper.move(-1100, step_delay)
    #open_stepper.stop()

    print("Spraying waffle iron with PAM...")
    # Spray the top of the waffle iron with PAM
    spray_servo.set_angle(150) 
    time.sleep(2)
    spray_servo.set_angle(-150)
    #time.sleep(2)


    # Spray the bottom of the waffle iron with PAM
    twist_servo.set_angle(50)
    time.sleep(1)

    spray_servo.set_angle(150) 
    time.sleep(2)
    spray_servo.set_angle(-150)
    time.sleep(2)

    twist_servo.set_angle(-50)
    time.sleep(1)
    
    '''
    print("Dispensing batter")
    # Dispense batter
    dispense_motor.forward()
    time.sleep(1.5)
    dispense_motor.stop()
    time.sleep(25)               # Dispense batter continuously for 5 seconds
    dispense_motor.backward()
    time.sleep(1.5)
    dispense_motor.stop()

    open_stepper.move(1200, step_delay)
    open_stepper.stop()
    print("Cooking waffle...")
    time.sleep(cook_time/2)    
    print("Halfway done")
    time.sleep(cook_time/2)   

    print("Flipping waffle...")
    flip_stepper.move(2600, step_delay)
    flip_stepper.stop()
    time.sleep(2)
    open_stepper.move(-1200, step_delay)
    open_stepper.stop()
    time.sleep(2)
    flip_stepper.move(-2600, step_delay)
    flip_stepper.stop()
    time.sleep(2)
    open_stepper.move(1100, step_delay)
    open_stepper.stop()
    time.sleep(2)
    '''


except KeyboardInterrupt:
    print("\nShutting down motors...")
finally:
    # Clean up all GPIO pins
    GPIO.cleanup()