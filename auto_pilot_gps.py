import time, cv2
import numpy as np   

from drivers_diff import Car
from vision import Vision
from lidar import Lidar
from sonar import Sonar
from gps import Gps
from gps_nav import GpsNav

car   = Car()
cam   = Vision()
lidar = Lidar()
sonar = Sonar()
gps   = Gps()
waypoints = [tuple(map(float, ln.split(','))) for ln in open('/home/pi/selfdrive/mission.txt')]
gps_nav   = GpsNav(gps, waypoints)

STATE = 'GPS'          # GPS / VISION / STOP_LIGHT / STOP_OBSTACLE
stop_timer = 0

try: 
    while True:
        # ---- sensors ----
        frame       = cam.get_frame()
        lane_angle  = cam.detect_lanes(frame)
        light       = cam.detect_traffic_light(frame)
        front_lidar = lidar.front_clearance()
        front_us    = sonar.front()
        front_clear = min(front_lidar, front_us) if front_us < 0.5 else front_lidar

        # ---- GPS layer ----
        gps_throttle, gps_steer = gps_nav.update()

        # ---- sensor confidence ----
        lane_conf = 1.0 - abs(lane_angle)   # crude: big angle = low conf
        if lane_conf < 0.3 and gps.fix and gps_throttle is not None:
            STATE = 'GPS'
        elif lane_conf >= 0.3:
            STATE = 'VISION'

        # ---- safety overrides (highest priority) ----
        if light == 'red':
            STATE = 'STOP_LIGHT'
        if front_clear < 0.25:
            STATE = 'STOP_OBSTACLE'

        # ---- act ----
        if STATE == 'GPS':
            car.steer(gps_steer)
            car.drive(gps_throttle)
        elif STATE == 'VISION':
            car.steer(lane_angle)
            car.drive(0.35 if front_clear > 0.80 else 0.20)
        else:
            car.stop()

        print(f"{STATE}  lat={gps.lat:.6f}  lon={gps.lon:.6f}  front={front_clear:.2f}")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping...")
    car.stop()
    lidar.close()