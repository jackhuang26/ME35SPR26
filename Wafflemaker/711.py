import RPi.GPIO as GPIO
from time import sleep

# Use physical pin numbering
GPIO.setmode(GPIO.BOARD)

# Define pins
PIN_A = 7
PIN_B = 11

# Set pins as output
GPIO.setup(PIN_A, GPIO.OUT)
GPIO.setup(PIN_B, GPIO.OUT)

def forward():
    print("Moving Forward")
    GPIO.output(PIN_A, GPIO.HIGH)
    GPIO.output(PIN_B, GPIO.LOW)

def backward():
    print("Moving Backward")
    GPIO.output(PIN_A, GPIO.LOW)
    GPIO.output(PIN_B, GPIO.HIGH)

def stop():
    print("Stopping")
    GPIO.output(PIN_A, GPIO.LOW)
    GPIO.output(PIN_B, GPIO.LOW)

try:
        #forward()
        backward()
        sleep(0.2)
        stop()

except KeyboardInterrupt:
    print("\nCleaning up GPIO...")
    stop()
    GPIO.cleanup()