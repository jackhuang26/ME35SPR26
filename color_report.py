# Color Report - scan a ball and report the color back to the user

# red - similar to green, but red is greater compared to green and blue is lower
# blue - red and green are about the same, blue is much higher
# green - b (1600) > r (1300-1400) > g (1200-1300)
#yellow - red is very high, blue is higher than green

import RPi.GPIO as GPIO
import time

# Assign GPIO pin numbers to variables

led = 37
s0 = 13
s1 = 15
s2 = 16
s3 = 18
sig = 22 #labeled "out" on your board
cycles = 50

# Setup GPIO and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led, GPIO.OUT)
GPIO.setup(s0, GPIO.OUT)
GPIO.setup(s1, GPIO.OUT)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Set frequency scaling
GPIO.output(s0, GPIO.HIGH)
GPIO.output(s1, GPIO.LOW)

def DetectColor():
    GPIO.output(led, GPIO.HIGH)
    # Detect red values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    time.sleep(0.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    red = cycles / duration

    # Detect blue values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    blue = cycles / duration

    # Detect green values
    GPIO.output(s2, GPIO.HIGH)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    green = cycles / duration

    GPIO.output(led, GPIO.LOW)

#    print("red value - ", red)
#    print("green value - ", green)
#    print("blue value - ", blue)

    return red,green,blue

def returnColor():
    r,g,b = DetectColor()
    
    if r > b:
        color = 'yellow'
    elif (b-r) > 400:
        color = 'blue'
    elif (r-g) < 180:
        color = 'green'
    elif r-g > 200:
        color = 'red'  
    else:
        return 'Unknown'
    return color
try:
    for y in range(1, 3):
        returnColor()        

except KeyboardInterrupt:
    print("\nExiting Program")
finally:
    print(returnColor())
    GPIO.cleanup()