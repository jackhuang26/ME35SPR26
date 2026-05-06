from servo_module import Servo
from dc_motor_module import DCMotor
from stepper_heat_module import StepperMotor
import RPi.GPIO as GPIO
import time


# Runs just the PAM spraying loop, ASSUMES WAFFLE IRON IS OPEN

GPIO.setmode(GPIO.BOARD)

spray_servo = Servo(18)
twist_servo = Servo(15)

step_delay = 0.01
open_pins = [31, 33, 35, 37]
open_stepper = StepperMotor(open_pins, power_level=100)

flip_pins = [32, 36, 38, 40]
flip_stepper = StepperMotor(flip_pins)

cook_time = 120  # Cook for 180 seconds or 3 minutes?

try:

     twist_servo.set_angle(20)
     time.sleep(2)

     # spray_servo.set_angle(0)
     # time.sleep(2)
     # spray_servo.set_angle(120)
     # time.sleep(1)
     # spray_servo.set_angle(0)
     # time.sleep(1)

     # twist_servo.set_angle(0)
     # time.sleep(2)

     # spray_servo.set_angle(0)
     # time.sleep(2)
     # spray_servo.set_angle(120)
     # time.sleep(1)
     # spray_servo.set_angle(0)
     # time.sleep(1)
     
     twist_servo.set_angle(20)
     time.sleep(2)
     #print("center")
     # time.sleep(2)
     # twist_servo.set_angle(50)
     # spray_servo.set_angle(-150)
     # time.sleep(2)
     # spray_servo.set_angle(150)
     # print("top")
     # time.sleep(2)
     # twist_servo.set_angle(0)
     # time.sleep(2)
     # twist_servo.set_angle(-100)
     # spray_servo.set_angle(-150)
     # time.sleep(2)
     # spray_servo.set_angle(150)
     # print("bottom")
     # time.sleep(2)
     # twist_servo.set_angle(0)



#     print("Spraying waffle iron with PAM...")
#     # Spray the top of the waffle iron with PAM
#     spray_servo.set_angle(-150)

#     spray_servo.set_angle(150) 
#     time.sleep(2)
#     spray_servo.set_angle(-150)
#     time.sleep(2)

#     twist_servo.set_angle(70)
#     # Spray the bottom of the waffle iron with PAM
#     twist_servo.set_angle(-70)
    
#     time.sleep(1)
#     spray_servo.set_angle(-150)

#     spray_servo.set_angle(200) 
#     time.sleep(2)
#     spray_servo.set_angle(-150)
#     time.sleep(2)

#     twist_servo.set_angle(50)
#     time.sleep(2)

except KeyboardInterrupt:
     print("aborted")
finally:
     GPIO.cleanup()
   
