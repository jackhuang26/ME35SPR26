import RPi.GPIO as GPIO
import time

class StepperMotor:
    def __init__(self, pins):
        """
        Initializes a 4-wire stepper motor.
        :param pins: A list or tuple of 4 physical BOARD pins [OUT1, OUT2, OUT3, OUT4]
        """
        self.pins = pins
        self.current_step = 0
        
        # The 4-step sequence from your original code
        self.sequence = [
            [1, 0, 1, 0], # Step 0
            [0, 1, 1, 0], # Step 1
            [0, 1, 0, 1], # Step 2
            [1, 0, 0, 1]  # Step 3
        ]

        # Setup GPIO
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        elif GPIO.getmode() != GPIO.BOARD:
            GPIO.setmode(GPIO.BOARD)

        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

    def _set_step(self, step_index):
        """Internal helper to set pins for a specific step in the sequence."""
        state = self.sequence[step_index]
        for i in range(4):
            GPIO.output(self.pins[i], state[i])

    def move(self, steps, delay):
        """
        Moves the motor a specific number of steps.
        Positive for one direction, negative for the other.
        """
        direction = 1 if steps > 0 else -1
        total_steps = abs(steps)

        for _ in range(total_steps):
            self._set_step(self.current_step)
            time.sleep(delay)
            
            # Update step index and wrap around (0 to 3)
            self.current_step = (self.current_step + direction) % 4

    def stop(self):
        """Turn off all coils to save power and prevent overheating."""
        for pin in self.pins:
            GPIO.output(pin, GPIO.LOW)