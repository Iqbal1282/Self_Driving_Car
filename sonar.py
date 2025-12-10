#!/usr/bin/env python3
import RPi.GPIO as IO
import time, threading, numpy as np

# ---------- pin map ----------
SONAR = dict(
    front={'trig': 5,  'echo': 6 },
    left={'trig': 16, 'echo': 20},
    right={'trig': 19, 'echo': 26}
)

for name, pins in SONAR.items():
    IO.setup(pins['trig'], IO.OUT)
    IO.setup(pins['echo'], IO.IN)
    IO.output(pins['trig'], 0)

# ---------- shared data ----------
dist_cm = {k: 999 for k in SONAR}   # latest reading

# ---------- worker ----------
def _sense(name, trig, echo):
    while True:
        IO.output(trig, 1); time.sleep(12e-6); IO.output(trig, 0)
        start = time.time()
        while IO.input(echo) == 0: start = time.time()
        while IO.input(echo) == 1: stop = time.time()
        cm = (stop - start) * 17150
        dist_cm[name] = max(0, round(cm, 1))
        time.sleep(0.04)   # 25 Hz per sensor

# ---------- start threads ----------
for name, pins in SONAR.items():
    threading.Thread(target=_sense, args=(name, pins['trig'], pins['echo']),
                     daemon=True).start()

# ---------- public API ----------
class Sonar:
    def front(self):      # metres
        return dist_cm['front'] / 100.
    def left(self):
        return dist_cm['left'] / 100.
    def right(self):
        return dist_cm['right'] / 100.
    def all(self):
        return {k: v/100. for k, v in dist_cm.items()}