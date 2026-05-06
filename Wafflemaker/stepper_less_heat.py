import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

OUT1 = 32
OUT2 = 36
OUT3 = 38
OUT4 = 40

GPIO.setup(OUT1, GPIO.OUT)
GPIO.setup(OUT2, GPIO.OUT)
GPIO.setup(OUT3, GPIO.OUT)
GPIO.setup(OUT4, GPIO.OUT)

# --- PWM setup (limits current) ---
pwm_freq = 1000  # Hz
pwm1 = GPIO.PWM(OUT1, pwm_freq)
pwm2 = GPIO.PWM(OUT2, pwm_freq)
pwm3 = GPIO.PWM(OUT3, pwm_freq)
pwm4 = GPIO.PWM(OUT4, pwm_freq)

pwm1.start(0)
pwm2.start(0)
pwm3.start(0)
pwm4.start(0)

# Reduce this if still overheating
POWER = 40  # percent (try 40–70)

step_delay = 0.005  # slightly slower = less current spikes

def set_step(a, b, c, d):
    pwm1.ChangeDutyCycle(a * POWER)
    pwm2.ChangeDutyCycle(b * POWER)
    pwm3.ChangeDutyCycle(c * POWER)
    pwm4.ChangeDutyCycle(d * POWER)

def release():
    set_step(0, 0, 0, 0)

def open_gripper(num_steps):
    sequence = [
        (1,0,1,0),
        (0,1,1,0),
        (0,1,0,1),
        (1,0,0,1)
    ]

    for i in range(num_steps):
        step = sequence[i % 4]
        set_step(*step)
        time.sleep(step_delay)

    release()  # 🔥 IMPORTANT

def close_gripper(num_steps):
    sequence = [
        (1,0,0,1),
        (0,1,0,1),
        (0,1,1,0),
        (1,0,1,0)
    ]

    for i in range(num_steps):
        step = sequence[i % 4]
        set_step(*step)
        time.sleep(step_delay)

    release()  # 🔥 IMPORTANT

try:
    while True:
        num_steps = int(input("Enter number of steps to move: "))

        if num_steps > 0:
            open_gripper(num_steps)
        else:
            close_gripper(abs(num_steps))

except KeyboardInterrupt:
    release()
    GPIO.cleanup()