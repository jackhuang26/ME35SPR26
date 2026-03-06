import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

# Define the GPIO pins for the L298N motor driver
# 11, 13, 15, 12
#OUT1 = 7
#OUT2 = 11
#OUT3 = 13
#OUT4 = 15

OUT1 = 29
OUT2 = 31
OUT3 = 32
OUT4 = 33

# Set the GPIO pins as output
GPIO.setup(OUT1, GPIO.OUT)
GPIO.setup(OUT2, GPIO.OUT)
GPIO.setup(OUT3, GPIO.OUT)
GPIO.setup(OUT4, GPIO.OUT)

# Set initial state of pins to low
GPIO.output(OUT1,GPIO.LOW)
GPIO.output(OUT2,GPIO.LOW)
GPIO.output(OUT3,GPIO.LOW)
GPIO.output(OUT4,GPIO.LOW)

step_delay = 0.02

def open_gripper(num_steps, step_delay):

    current_step = 0
        
    for x in range(num_steps):
        if current_step == 0:
            GPIO.output(OUT1,GPIO.HIGH)
            GPIO.output(OUT2,GPIO.LOW)
            GPIO.output(OUT3,GPIO.HIGH)
            GPIO.output(OUT4,GPIO.LOW)
            time.sleep(step_delay)
            #print("step 0")
        elif current_step == 1:
            GPIO.output(OUT1,GPIO.LOW)
            GPIO.output(OUT2,GPIO.HIGH)
            GPIO.output(OUT3,GPIO.HIGH)
            GPIO.output(OUT4,GPIO.LOW)
            time.sleep(step_delay)
            #print("step 1")
        elif current_step == 2:
            GPIO.output(OUT1,GPIO.LOW)
            GPIO.output(OUT2,GPIO.HIGH)
            GPIO.output(OUT3,GPIO.LOW)
            GPIO.output(OUT4,GPIO.HIGH)
            time.sleep(step_delay)
        elif current_step == 3:
            GPIO.output(OUT1,GPIO.HIGH)
            GPIO.output(OUT2,GPIO.LOW)
            GPIO.output(OUT3,GPIO.LOW)
            GPIO.output(OUT4,GPIO.HIGH)
            time.sleep(step_delay)
        if current_step == 3:
            current_step = 0
            continue 
        current_step = current_step + 1
        
def close_gripper(num_steps, step_delay):

    current_step = 3
        
    for x in range(num_steps):
        if current_step == 0:
            GPIO.output(OUT1,GPIO.HIGH)
            GPIO.output(OUT2,GPIO.LOW)
            GPIO.output(OUT3,GPIO.HIGH)
            GPIO.output(OUT4,GPIO.LOW)
            time.sleep(step_delay)
            #print("step 0")
        elif current_step == 1:
            GPIO.output(OUT1,GPIO.LOW)
            GPIO.output(OUT2,GPIO.HIGH)
            GPIO.output(OUT3,GPIO.HIGH)
            GPIO.output(OUT4,GPIO.LOW)
            time.sleep(step_delay)
            #print("step 1")
        elif current_step == 2:
            GPIO.output(OUT1,GPIO.LOW)
            GPIO.output(OUT2,GPIO.HIGH)
            GPIO.output(OUT3,GPIO.LOW)
            GPIO.output(OUT4,GPIO.HIGH)
            time.sleep(step_delay)
        elif current_step == 3:
            GPIO.output(OUT1,GPIO.HIGH)
            GPIO.output(OUT2,GPIO.LOW)
            GPIO.output(OUT3,GPIO.LOW)
            GPIO.output(OUT4,GPIO.HIGH)
            time.sleep(step_delay)
        if current_step == 0:
            current_step = 3
            continue 
        current_step = current_step + -1

try:

    while True:

        num_steps = int(input("Enter number of steps to move: "))

        if num_steps > 0:
            open_gripper(num_steps, step_delay)
        else:
            num_steps = abs(num_steps)
            close_gripper(num_steps, step_delay)

except KeyboardInterrupt:
    GPIO.cleanup()


