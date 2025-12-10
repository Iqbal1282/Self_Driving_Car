#!/usr/bin/env python3
import numpy as np, threading, time
from rplidar import RPLidar

class Lidar:
    def __init__(self, port='/dev/ttyS0', safe_stop=0.30):   # 30 cm
        self.lidar = RPLidar(port)
        self.lidar.start_motor()
        self.scan = []          # last complete 360° scan
        self.safe_stop = safe_stop
        self.stop_flag = False
        threading.Thread(target=self._grab, daemon=True).start()

    def _grab(self):
        for meas in self.lidar.iter_scans(max_buf_meas=800):
            self.scan = meas        # list of (quality, angle, distance_mm)
            if self.stop_flag:
                break
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    # ---------- helpers ----------
    def polar_to_cart(self, angle_deg, dist_mm):
        rad = np.deg2rad(angle_deg)
        return dist_mm*np.sin(rad), dist_mm*np.cos(rad)   # x,y  (x=right, y=forward)

    def front_clearance(self, half_width_deg=35):
        """return min distance in a 70° frontal cone"""
        if not self.scan:
            return 999
        front = [d for (_,a,d) in self.scan
                 if (360-half_width_deg <= a or a <= half_width_deg) and d>0]
        return min(front)/1000.0 if front else 999   # metres

    def gap_detect(self, min_gap_m=0.50, car_width_m=0.22):
        """return angle (deg) of largest free gap, None if none"""
        if not self.scan:
            return None
        # build 1-D occupancy vector  (1° resolution)
        occ = np.full(360, np.inf)
        for (_,a,d) in self.scan:
            if d>0:
                occ[int(a)] = d/1000.0
        # sliding window
        best_start, best_len = 0, 0
        win = int(np.ceil(np.rad2deg(2*np.arctan(car_width_m/2/0.30))))  # ~42° at 30 cm
        for start in range(360):
            length = 0
            for i in range(start, start+win):
                if occ[i%360] >= min_gap_m:
                    length += 1
                else:
                    break
            if length > best_len:
                best_start, best_len = start, length
        if best_len < win*0.7:
            return None
        return (best_start + win//2) % 360   # centre of gap

    def close(self):
        self.stop_flag = True


    def grid(self, width_m=2.0, height_m=2.0, res=0.02):
        """return 2-D numpy array (occupancy 0-1) local grid centered on car"""
        w = int(width_m/res); h = int(height_m/res)
        grid = np.zeros((h, w), dtype=np.float32)
        cx = w//2; cy = h-1
        for _,a,d_mm in self.scan:
            if d_mm==0: continue
            x,y = self.polar_to_cart(a, d_mm)
            px = int(cx + x/1000/res)
            py = int(cy - y/1000/res)
            if 0<=px<w and 0<=py<h:
                grid[py, px] = 1
        return grid