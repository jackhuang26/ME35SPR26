# Core opencv code provided by Einsteinium Studios
# Revisions to work with Pi Camera v3 by Briana Bouchard
# Modified for HSV color-based line following with motor control
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import controls
import time
import RPi.GPIO as GPIO

# --- MOTOR CONFIGURATION ---
# Motor Speed Variables (0-100)
BASE_SPEED = 15       # Speed when moving straight
TURN_SPEED_HIGH = 40  # Speed of the outer wheel during a turn
TURN_SPEED_LOW = 0    # Speed of the inner wheel during a turn

# Pin Definitions (Board Numbering)
ENA = 22
IN1 = 24
IN2 = 26
ENB = 19
IN3 = 21
IN4 = 23

direction = 0

# --- GPIO SETUP ---
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Setup Motor Pins
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Initialize PWM
motor_left = GPIO.PWM(ENA, 50)
motor_right = GPIO.PWM(ENB, 50)
motor_left.start(0)
motor_right.start(0)

# --- MOTOR FUNCTIONS ---
def stop_motors():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    motor_left.ChangeDutyCycle(0)
    motor_right.ChangeDutyCycle(0)

def move_forward(speed):
    # Left Motor Forward (reversed)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    # Right Motor Forward (reversed)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
    motor_left.ChangeDutyCycle(speed)
    motor_right.ChangeDutyCycle(speed)

def turn_left():
    # To turn LEFT: Right motor goes fast, Left motor stops/slows (reversed)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
    motor_left.ChangeDutyCycle(TURN_SPEED_LOW)
    motor_right.ChangeDutyCycle(TURN_SPEED_HIGH)

def turn_right():
    # To turn RIGHT: Left motor goes fast, Right motor stops/slows (reversed)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
    motor_left.ChangeDutyCycle(TURN_SPEED_HIGH)
    motor_right.ChangeDutyCycle(TURN_SPEED_LOW)

# --- COLOR DETECTION CONFIGURATION ---
# Define color ranges in HSV
COLOR_RANGES = {
    'red': [(50, 40, 100), (80, 60, 255)],      # Red (lower range)
    'red2': [(100, 100, 100), (180, 255, 255)],  # Red (upper range - wraps around)
    'green': [(65, 40, 90), (80, 65, 120)],   # Green
    'blue': [(100, 100, 100), (130, 255, 255)],  # Blue
    'yellow': [(100, 100, 100), (100, 255, 255)],  # Yellow
    'orange': [(100, 100, 100), (20, 255, 255)],  # Orange
    'purple': [(130, 100, 100), (160, 255, 255)] # Purple
}

def create_color_mask(hsv_image, color_name):
    """
    Create a binary mask for the specified color.
    Handles red color specially since it wraps around the HSV hue spectrum.
    """
    if color_name == 'red':
        # Red requires two ranges because it wraps around 0/180
        lower1 = np.array(COLOR_RANGES['red'][0])
        upper1 = np.array(COLOR_RANGES['red'][1])
        lower2 = np.array(COLOR_RANGES['red2'][0])
        upper2 = np.array(COLOR_RANGES['red2'][1])
        
        mask1 = cv2.inRange(hsv_image, lower1, upper1)
        mask2 = cv2.inRange(hsv_image, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        lower = np.array(COLOR_RANGES[color_name][0])
        upper = np.array(COLOR_RANGES[color_name][1])
        mask = cv2.inRange(hsv_image, lower, upper)
    
    return mask

# --- CAMERA SETUP ---
# Set which color to follow (change this to track different colors)
TARGET_COLOR = 'green'  # Options: 'red', 'green', 'blue', 'yellow', 'orange', 'purple'

picam2 = Picamera2()  # assigns camera variable
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})  # sets auto focus mode
picam2.start()  # activates camera
time.sleep(1)  # wait to give camera time to start up

print(f"Camera-based Line Follower Started - Following {TARGET_COLOR} line")
print("Press Ctrl+C to stop.")

try:
    while True:
        
        # Display camera input
        image = picam2.capture_array("main")
        cv2.imshow('img', image)
    
        # Crop the image
        crop_img = image[0:1000, 0:1000]
    
        # Convert to HSV color space
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    
        # Create mask for target color
        mask = create_color_mask(hsv, TARGET_COLOR)
        
        # Apply Gaussian blur to reduce noise
        blur = cv2.GaussianBlur(mask, (5, 5), 0)
    
        # Find the contours of the frame
        contours, hierarchy = cv2.findContours(blur.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
        # Find the biggest contour (if detected)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)  # determine moment - weighted average of intensities
            
            if int(M['m00']) != 0:
                cx = int(M['m10'] / M['m00'])  # find x component of centroid location
                cy = int(M['m01'] / M['m00'])  # find y component of centroid location
            else:
                print("Centroid calculation error, looping to acquire new values")
                continue
                
            cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)  # display vertical line at x value of centroid
            cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)  # display horizontal line at y value of centroid
    
            cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 2)  # display green lines for all contours
            
            # --- MOTOR CONTROL BASED ON CENTROID POSITION ---
            # Determine location of centroid in x direction and control motors
            # 0 left, 1 is right

            if cx >= 450:
                print("Turn Right!")
                turn_right()
                direction = 1
    
            elif cx < 550 and cx > 450:
                print("On Track!")
                move_forward(BASE_SPEED)
    
            elif cx <= 550:
                print("Turn Left!")
                turn_left()
                direction = 0
    
        else:
            print("I don't see the line")
            stop_motors()
            if direction == 1:
                turn_right()
            elif direction == 0:
                turn_left()
    
        # Display the resulting frames
        cv2.imshow('frame', crop_img)
        cv2.imshow('mask', mask)  # Show the color mask for debugging
        
        # Show image for 1 ms then continue to next image
        cv2.waitKey(1)
        
except KeyboardInterrupt:
    print('\nStopping Robot...')
    stop_motors()
    cv2.destroyAllWindows()
    picam2.stop()
    GPIO.cleanup()
    print("Cleaned up GPIO and camera.")