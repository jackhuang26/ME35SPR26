import RPi.GPIO as GPIO
import time

class Servo:
    def __init__(self, pin, frequency=50):
        self.pin = pin
        self.frequency = frequency
        
        # Ensure GPIO mode is set to BOARD
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        elif GPIO.getmode() != GPIO.BOARD:
            print("Warning: GPIO mode was already set to BCM. Overriding to BOARD.")
            GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.frequency)
        self.pwm.start(0)

    def set_angle(self, angle):
        """
        Moves the servo to a specific angle (0-180).
        Calculation: Duty Cycle = (Angle / 18) + 2
        """
        # Constrain angle to physical limits
        angle = max(0, min(240, angle))
        
        duty = angle / 24 + 2
        GPIO.output(self.pin, True)
        self.pwm.ChangeDutyCycle(duty)
        
        # Allow time for the motor to physically reach the position
        time.sleep(0.3) 
        
        # Stop signal to prevent jitter/buzzing
        GPIO.output(self.pin, False)
        self.pwm.ChangeDutyCycle(0)

    def stop(self):
        """Stops the PWM signal for this specific servo."""
        self.pwm.stop()

    def __del__(self):
        """Ensures PWM stops if the object is deleted."""
        self.stop()

def global_cleanup():
    """Call this at the very end of your main script."""
    GPIO.cleanup()