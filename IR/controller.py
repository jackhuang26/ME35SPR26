import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

# IR sensor pins 
sensor_pins = [11, 13, 15, 16, 18]
sensor_weights = [-3, -1, 0, 1, 3]

# S2 —> error = 0 (perfect)
# S1 —> error = -1 (line left)
# S3 —> error = +1 (line right)

for pin in sensor_pins:
    GPIO.setup(pin, GPIO.IN)

# motor pins 
LEFT_ENA = 22
LEFT_IN1 = 24
LEFT_IN2 = 26

RIGHT_ENB = 36
RIGHT_IN3 = 38
RIGHT_IN4 = 40

motor_pins = [LEFT_ENA, LEFT_IN1, LEFT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4]

for pin in motor_pins: 
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

motor_left = GPIO.PWM(LEFT_ENA, 100)
motor_right = GPIO.PWM(RIGHT_ENB, 100)

motor_left.start(0)
motor_right.start(0)

base_speed = 18
max_speed = 50

# controller
Kp = 30.0
Kd = 15.0
Ki = 0.0

err_prev = 0
err_int = 0

def read_sensor_error():
    readings = []
    for pin in sensor_pins:
        readings.append(GPIO.input(pin))

    normalized = []
    for r in readings: 
        normalized.append(1 - r)

    weighted_sum = 0
    black_detected = 0

    for val, weight in zip(normalized, sensor_weights):
        weighted_sum += val * weight
        black_detected += val

    if black_detected == 0:
        return None         # line has been lost 
    
    error = weighted_sum / black_detected

    return error

def pid_control(error):
    global err_prev, err_int 

    err_int += error
    err_dot = error - err_prev

    proportional = Kp * error
    derivative = Kd * err_dot
    integral = Ki * err_int
    
    output = proportional + derivative + integral

    err_prev = error
    return output 

def motor_control(pid_output):
    left_speed = base_speed - pid_output
    right_speed = base_speed + pid_output

    left_speed = max(min(left_speed, max_speed), 0)
    right_speed = max(min(right_speed, max_speed), 0)

    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.HIGH)
    GPIO.output(RIGHT_IN4, GPIO.LOW)

    motor_left.ChangeDutyCycle(left_speed)
    motor_right.ChangeDutyCycle(right_speed)

try: 
    while True: 
        error = read_sensor_error()

        if error is None: 
            print("\nLine lost...")
            motor_left.ChangeDutyCycle(0)
            motor_right.ChangeDutyCycle(0)
            continue
            
        pid_output = pid_control(error)
        motor_control(pid_output)

        time.sleep(0.01)
    
except KeyboardInterrupt: 
    print("\nExiting Program")

finally:
    motor_left.stop()
    motor_right.stop()

    GPIO.cleanup()
