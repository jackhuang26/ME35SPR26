from lcd_i2c import I2CLCD
from servo_module import Servo
from dc_motor_module import DCMotor
from stepper_heat_module import StepperMotor
import airtable_module as airtable
import RPi.GPIO as GPIO
import time

# LCD setup
#lcd = I2CLCD(address=0x27, bus_num=1, cols=16, rows=2)
#lcd.backlight_on()

# Hardware setup
spray_servo = Servo(18)
twist_servo = Servo(16)

dispense_motor = DCMotor(7, 11)

step_delay = 0.01
open_pins = [31, 33, 35, 37]
open_stepper = StepperMotor(open_pins, power_level=100)

flip_pins = [32, 36, 38, 40]
flip_stepper = StepperMotor(flip_pins)

cook_time = 5  # Cook for 120 seconds / 2 minutes

def cook_with_timer(seconds):

    for remaining in range(seconds, 0, -1):
        mins = remaining // 60
        secs = remaining % 60
        #lcd.write_lines([
        #    "Cooking waffle...",
        #    f"Time: {mins:02d}:{secs:02d}    ",
        #])
        print(f"Cooking waffle... Time: {mins:02d}:{secs:02d}")
        time.sleep(1)

def make_a_waffle():

    print("Opening waffle iron...")
    open_stepper.move(-1100, step_delay)
    open_stepper.stop()

    # Spray with PAM
    print("Spraying waffle iron with PAM...")
    spray_servo.set_angle(150)
    time.sleep(2)
    spray_servo.set_angle(-150)
    time.sleep(2)

    twist_servo.set_angle(50)
    time.sleep(1)
    spray_servo.set_angle(150)
    time.sleep(2)
    spray_servo.set_angle(-150)
    time.sleep(2)
    twist_servo.set_angle(-50)
    time.sleep(1)
    
    # Dispense batter
    #lcd.write_lines([
    #    "Pouring batter",
    #])
    print("Dispensing batter")
    dispense_motor.forward()
    time.sleep(2)
    dispense_motor.stop()
    time.sleep(11)
    dispense_motor.backward()
    time.sleep(2)
    dispense_motor.stop()

    open_stepper.move(1175, step_delay)
    open_stepper.stop()

    # Cook with LCD countdown
    print("Cooking waffle")
    cook_with_timer(cook_time)

    # Flip
    #lcd.write_lines([
    #    "Flipping waffle",
    #])

    print("Flipping waffle...")
    flip_stepper.move(2600, step_delay)
    flip_stepper.stop()
    time.sleep(2)
    open_stepper.move(-1175, step_delay)
    open_stepper.stop()
    time.sleep(2)
    flip_stepper.move(-2600, step_delay)
    flip_stepper.stop()
    time.sleep(2)
    open_stepper.move(1100, step_delay)
    open_stepper.stop()
    time.sleep(2)

    #lcd.write_lines([
    #    "Waffle is ready!",
    #])

try:
    #while True:
        #lcd.write_lines([
        #    "Waffle Machine  ",
        #    "Waiting...      ",
        #])
        #print("Waiting for Airtable signal...")
        #airtable.wait_until_ready("waffle")
        print("Received signal to start waffle process.")
        airtable.update_status("waffle", "executing")
        make_a_waffle()
        #airtable.update_status("waffle", "success")
        #time.sleep(5)
        #airtable.update_status("waffle", "waiting")

except KeyboardInterrupt:

    print("\nShutting down motors...")
    #lcd.clear()
    #lcd.backlight_off()

finally:
    GPIO.cleanup()
