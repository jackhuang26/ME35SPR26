from servo_module import Servo
from dc_motor_module import DCMotor
import RPi.GPIO as GPIO
import time

spray_servo = Servo(18)
twist_servo = Servo(16)

dispense_motor = DCMotor(7, 11)

try:

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

    twist_servo.set_angle(0)
    time.sleep(1)

    '''
    dispense_motor.forward()
    time.sleep(3)
    dispense_motor.stop()
    time.sleep(2)
    dispense_motor.backward()
    time.sleep(3)
    dispense_motor.stop()
    '''


except KeyboardInterrupt:
    print("\nShutting down motors...")
finally:
    # Clean up all GPIO pins
    GPIO.cleanup()