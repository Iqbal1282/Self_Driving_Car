#!/usr/bin/env python3
import time, cv2
from drivers import Car
from vision import Vision
from lidar  import Lidar
from sonar import Sonar 
import numpy as np   

car   = Car()
cam   = Vision()
lidar = Lidar(safe_stop=0.30)
sonar = Sonar()

STATE = 'CRUISE'   # CRUISE | STOP_LIGHT | STOP_OBSTACLE
stop_timer = 0

try:
    while True:
        # ---- raw sensors ----
        frame       = cam.get_frame()
        lane_angle  = cam.detect_lanes(frame)
        light       = cam.detect_traffic_light(frame)
        front_lidar = lidar.front_clearance()          # metres
        gap_deg     = lidar.gap_detect()
        us          = sonar.all()                      # dict front/left/right (m)

        # ---- fusion layer ----
        # 1) ultra-short-range curb / wall
        front_us = us['front']
        side_min = min(us['left'], us['right'])

        # 2) conservative “front_clear”
        front_clear = min(front_lidar, front_us) if front_us < 0.50 else front_lidar

        # ---- state machine ----
        if light == 'red':
            STATE = 'STOP_LIGHT'
        elif light == 'green' and STATE == 'STOP_LIGHT':
            STATE = 'CRUISE'

        if front_clear < 0.25:          # 25 cm fused
            STATE = 'STOP_OBSTACLE'
        elif STATE == 'STOP_OBSTACLE' and front_clear > 0.40:
            STATE = 'CRUISE'

        # 3) side curb / garage-wall mode (optional)
        if side_min < 0.08 and front_clear > 0.30:   # 8 cm side, 30 cm front
            STATE = 'CURB'
            car.drive(0.15)            # creep
            car.steer(0.0)
            continue                   # skip normal cruise

        # ---- act ----
        if STATE == 'CRUISE':
            # steering: prefer CNN, fall back to LiDAR gap if wall dead-ahead
            if front_lidar < 0.80 and abs(lane_angle) > 0.5 and gap_deg is not None:
                steer_cmd = np.clip((gap_deg - 180) / 90, -1, 1)
            else:
                steer_cmd = lane_angle
            car.steer(steer_cmd)
            car.drive(0.35 if front_clear > 0.80 else 0.20)
        else:
            car.stop()

        # ---- console debug ----
        print(f"{time.time():.1f}  {STATE}  front_l={front_lidar:.2f}  front_u={front_us:.2f}  angle={lane_angle:.2f}")

        time.sleep(0.05)   # 20 Hz

except KeyboardInterrupt:
    print("Stopping...")
    car.stop()
    lidar.close()