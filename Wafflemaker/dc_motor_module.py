import RPi.GPIO as GPIO

class DCMotor:
    def __init__(self, pin_a, pin_b, enable_pin):
        """
        Initializes a DC motor.
        :param pin_a: Physical BOARD pin for direction A
        :param pin_b: Physical BOARD pin for direction B
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.enable = enable_pin

        # Set up GPIO mode if not already set
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        elif GPIO.getmode() != GPIO.BOARD:
            # Gentle nudge if the mode is inconsistent
            print("Warning: GPIO mode was set to BCM. Switching to BOARD for this module.")
            GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.pin_a, GPIO.OUT)
        GPIO.setup(self.pin_b, GPIO.OUT)
        GPIO.setup(self.enable, GPIO.OUT)
        self.stop() # Ensure motor is off initially

    def forward(self):
        """Sets the motor to rotate in direction A."""
        GPIO.output(self.pin_a, GPIO.HIGH)
        GPIO.output(self.pin_b, GPIO.LOW)
        GPIO.output(self.enable, GPIO.HIGH)

    def backward(self):
        """Sets the motor to rotate in direction B."""
        GPIO.output(self.pin_a, GPIO.LOW)
        GPIO.output(self.pin_b, GPIO.HIGH)
        GPIO.output(self.enable, GPIO.HIGH)

    def stop(self):
        """Stops the motor by cutting power to both pins."""
        GPIO.output(self.pin_a, GPIO.LOW)
        GPIO.output(self.pin_b, GPIO.LOW)
        GPIO.output(self.enable, GPIO.LOW)

def cleanup():
    """Resets all GPIO pins."""
    GPIO.cleanup()