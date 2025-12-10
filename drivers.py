#! /usr/bin/env python3 
# /home/pi/selfdrive/
import RPi.GPIO as IO 
import numpy as np 


IO.setmode(IO.BCM)
IO.setwarnings(False) 


# Motor pins 
PWMA, AIN1, AIN2, STBY = 12, 23, 24, 25 
IO.setup([PWMA, AIN1, AIN2, STBY], IO.OUT)
motor_pwm =  IO.PWM(PWMA, 1000)
motor_pwm.start(0)


# servo pin 
SERVO = 13 
IO.setup(SERVO, IO.OUT)
servo_pwm = IO.PWM(SERVO, 50)
servo_pwm.start(7.5) # center 

class Car: 
    def __init__(self): 
        self.drive(0)
        self.steer(0)

    def drive(self, speed): # speed (-1, 1)

        speed = np.clip(speed, -1, 1)
        IO.output(STBY, 1)
        
        if speed >= 0: 
            IO.output([AIN1, AIN2], (1, 0))
        else:
            IO.output([AIN1, AIN2], (0, 1))

        
        motor_pwm.ChangeDutyCycle(abs(speed)*100)

    
    def steer(self, angle): 
        # angle (-1, 1)

        angle = np.clip(angle, -1, 1)
        duty = 7.5 - angle*2.2 

        servo_pwm.ChangeDutyCycle(duty) 


    
    def stop(self): 
        self.drive(0) 
