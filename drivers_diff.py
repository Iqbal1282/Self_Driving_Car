#!/usr/bin/env python3
import RPi.GPIO as IO
import numpy as np

IO.setmode(IO.BCM)
IO.setwarnings(False)

# ---------- LEFT motor ----------
ENA, IN1, IN2 = 12, 23, 24
IO.setup([ENA, IN1, IN2], IO.OUT)
pwmL = IO.PWM(ENA, 1000)   # 1 kHz
pwmL.start(0)

# ---------- RIGHT motor ----------
ENB, IN3, IN4 = 20, 16, 21
IO.setup([ENB, IN3, IN4], IO.OUT)
pwmR = IO.PWM(ENB, 1000)
pwmR.start(0)

def set_side(pwm, in1, in2, speed):
    """helper: speed -1..1"""
    speed = np.clip(speed, -1, 1)
    if speed >= 0:
        IO.output([in1, in2], (1, 0))
    else:
        IO.output([in1, in2], (0, 1))
    pwm.ChangeDutyCycle(abs(speed) * 100)

class Car:
    def __init__(self):
        self.drive(0)
        self.steer(0)

    # throttle: -1 (reverse) .. 0 .. +1 (forward)
    def drive(self, throttle):
        throttle = np.clip(throttle, -1, 1)
        # both wheels same speed for pure forward/back
        set_side(pwmL, IN1, IN2, throttle)
        set_side(pwmR, IN3, IN4, throttle)

    # angle: -1 (pivot left) .. 0 (straight) .. +1 (pivot right)
    def steer(self, angle):
        angle = np.clip(angle, -1, 1)
        # differential drive: reduce inner wheel
        if angle < 0:   # left turn
            set_side(pwmL, IN1, IN2, 0.3 * (1 + angle))  # slower left
            set_side(pwmR, IN3, IN4, 0.3)                # right base
        elif angle > 0: # right turn
            set_side(pwmL, IN1, IN2, 0.3)                # left base
            set_side(pwmR, IN3, IN4, 0.3 * (1 - angle))  # slower right
        else:           # straight
            set_side(pwmL, IN1, IN2, 0.3)
            set_side(pwmR, IN3, IN4, 0.3)

    def stop(self):
        set_side(pwmL, IN1, IN2, 0)
        set_side(pwmR, IN3, IN4, 0)