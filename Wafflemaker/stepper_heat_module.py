import RPi.GPIO as GPIO
import time

class StepperMotor:
    def __init__(self, pins, power_level=40):
        """
        Initializes a 4-wire stepper motor with PWM for heat reduction.
        :param pins: A list or tuple of 4 physical BOARD pins [OUT1, OUT2, OUT3, OUT4]
        :param power_level: Duty cycle percentage (0-100) to limit current.
        """
        self.pins = pins
        self.power_level = power_level
        self.current_step = 0
        self.pwm_freq = 1000  # 1kHz frequency
        self.pwms = []
        
        # Standard 4-step sequence
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
            # Initialize PWM for each pin
            p = GPIO.PWM(pin, self.pwm_freq)
            p.start(0)
            self.pwms.append(p)

    def _set_step(self, step_index):
        """Internal helper to set PWM duty cycles for a specific step."""
        state = self.sequence[step_index]
        for i in range(4):
            # Apply power level only if the sequence state is 1 (High)
            duty_cycle = state[i] * self.power_level
            self.pwms[i].ChangeDutyCycle(duty_cycle)

    def move(self, steps, delay):
        """
        Moves the motor a specific number of steps.
        Automatically stops/releases after movement to prevent heat buildup.
        """
        direction = 1 if steps > 0 else -1
        total_steps = abs(steps)

        for _ in range(total_steps):
            self._set_step(self.current_step)
            time.sleep(delay)
            
            # Update step index and wrap around (0 to 3)
            self.current_step = (self.current_step + direction) % 4
        
        # Automatically turn off coils after movement is complete
        if self.pins == [31, 33, 35, 37]:
            return
        else:
            self.stop()

    def stop(self):
        """Sets all PWM duty cycles to 0 to save power and eliminate heat."""
        for p in self.pwms:
            p.ChangeDutyCycle(0)

    def cleanup(self):
        """Stops PWM and cleans up GPIO settings."""
        for p in self.pwms:
            p.stop()
        GPIO.cleanup()