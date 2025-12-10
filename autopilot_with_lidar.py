#!/usr/bin/env python3
import time, cv2
from drivers import Car
from vision import Vision
from lidar  import Lidar
import numpy as np   

car   = Car()
cam   = Vision()
lidar = Lidar(safe_stop=0.30)

STATE = 'CRUISE'   # CRUISE | STOP_LIGHT | STOP_OBSTACLE
stop_timer = 0

try:
    while True:
        # ---- sensor layer ----
        frame      = cam.get_frame()
        lane_angle = cam.detect_lanes(frame)
        light      = cam.detect_traffic_light(frame)
        front_m    = lidar.front_clearance()     # metres
        gap_deg    = lidar.gap_detect()

        # ---- decision layer ----
        # 1) traffic-light override
        if light == 'red':
            STATE = 'STOP_LIGHT'
        elif light == 'green' and STATE == 'STOP_LIGHT':
            STATE = 'CRUISE'

        # 2) LiDAR safety stop
        if front_m < 0.60:          # 60 cm
            STATE = 'STOP_OBSTACLE'
        elif STATE == 'STOP_OBSTACLE' and front_m > 0.80:
            STATE = 'CRUISE'

        # 3) steering fusion
        if STATE == 'CRUISE':
            # If CNN wants to turn into a wall, trust LiDAR gap
            if front_m < 1.0 and abs(lane_angle) > 0.5 and gap_deg is not None:
                # convert gap angle to steering (-1..1)
                steer_cmd = np.clip((gap_deg - 180) / 90, -1, 1)
            else:
                steer_cmd = lane_angle
            car.steer(steer_cmd)
            car.drive(0.40 if front_m > 1.0 else 0.25)  # slow down when close
        else:
            car.stop()

        # ---- debug console ----
        print(f"state={STATE}  front={front_m:.2f}m  gap={gap_deg}°  lane_angle={lane_angle:.2f}")

        time.sleep(0.05)   # 20 Hz

except KeyboardInterrupt:
    print("Stopping...")
    car.stop()
    lidar.close()