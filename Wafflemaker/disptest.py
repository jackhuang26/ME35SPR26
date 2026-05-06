from servo_module import Servo
from dc_motor_module import DCMotor
from stepper_heat_module import StepperMotor
import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BOARD)

# Initialize dispenser motor
GPIO.setup(8, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)

GPIO.output(8, GPIO.LOW)
GPIO.output(11, GPIO.LOW)
GPIO.output(12, GPIO.LOW)

dispenser = GPIO.PWM(12, 100)
dispenser.start(100)

try:
    print("Dispensing batter")
    # Dispense batter
    #dispenser.ChangeDutyCycle(100)  # Start dispensing
    GPIO.output(8, GPIO.HIGH)
    GPIO.output(11, GPIO.LOW)
    time.sleep(1.65)
    GPIO.output(8, GPIO.LOW)
    GPIO.output(11, GPIO.LOW)
    #dispenser.ChangeDutyCycle(0)  # Stop dispensing
    time.sleep(26)
    #dispenser.ChangeDutyCycle(100)  # Start dispensing
    GPIO.output(8, GPIO.LOW)
    GPIO.output(11, GPIO.HIGH)
    time.sleep(1.65)
    #dispenser.ChangeDutyCycle(0)  # Stop dispensing
    GPIO.output(8, GPIO.LOW)
    GPIO.output(11, GPIO.LOW)
    time.sleep(5)

except KeyboardInterrupt:
    print("\nShutting down motors...")
finally:
    # Clean up all GPIO pins
    GPIO.cleanup()