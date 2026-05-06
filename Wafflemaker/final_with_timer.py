from lcd_i2c import I2CLCD
from servo_module import Servo
from dc_motor_module import DCMotor
from stepper_heat_module import StepperMotor
import airtable_module as airtable
import RPi.GPIO as GPIO
import time


# LCD setup
lcd = I2CLCD(address=0x27, bus_num=1, cols=20, rows=2)
lcd.backlight_on()

# Hardware setup
spray_servo = Servo(18)
twist_servo = Servo(16)

dispense_motor = DCMotor(7, 11)

step_delay = 0.01
open_pins = [31, 33, 35, 37]
open_stepper = StepperMotor(open_pins, power_level=100)

flip_pins = [32, 36, 38, 40]
flip_stepper = StepperMotor(flip_pins)

cook_time = 120  # Cook for 120 seconds / 2 minutes

def cook_with_timer(seconds):
    for remaining in range(seconds, 0, -1):
        mins = remaining // 60
        secs = remaining % 60
        lcd.write_lines([
            "  COOKING WAFFLE... ",
            f"  Time left: {mins:02d}:{secs:02d}  ",
        ])
        time.sleep(1)
    lcd.write_lines([
        "  WAFFLE IS DONE!  ",
        "  Flipping now...  ",
    ])

def make_a_waffle():
    lcd.write_lines([
        "  STARTING WAFFLE  ",
        "   process...      ",
    ])
    time.sleep(2)

    # Open the waffle iron
    lcd.write_lines([
        "  Opening iron...  ",
        "                   ",
    ])
    print("Opening waffle iron...")
    open_stepper.move(-1100, step_delay)
    open_stepper.stop()

    # Spray with PAM
    lcd.write_lines([
        "  Spraying PAM...  ",
        "                   ",
    ])
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
    lcd.write_lines([
        " Dispensing batter ",
        "   please wait...  ",
    ])
    print("Dispensing batter")
    dispense_motor.forward()
    time.sleep(1.5)
    dispense_motor.stop()
    time.sleep(13)
    dispense_motor.backward()
    time.sleep(1.5)
    dispense_motor.stop()

    open_stepper.move(1200, step_delay)
    open_stepper.stop()

    # Cook with LCD countdown
    print("Cooking waffle...")
    cook_with_timer(cook_time)

    # Flip
    lcd.write_lines([
        " Flipping waffle.. ",
        "   please wait...  ",
    ])
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

    lcd.write_lines([
        " WAFFLE IS READY!  ",
        "  Enjoy your waffle",
    ])

try:
    while True:
        lcd.write_lines([
            "  WAFFLE MACHINE   ",
            "  Waiting for order",
        ])
        print("Waiting for Airtable signal...")
        airtable.wait_until_ready("waffle")
        print("Received signal to start waffle process.")
        airtable.update_status("waffle", "executing")
        make_a_waffle()
        airtable.update_status("waffle", "success")
        time.sleep(5)
        airtable.update_status("waffle", "waiting")

except KeyboardInterrupt:
    print("\nShutting down motors...")
    lcd.clear()
    lcd.backlight_off()
finally:
    GPIO.cleanup()
