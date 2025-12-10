#!/usr/bin/env python3
import numpy as np

class GpsNav:
    def __init__(self, gps, waypoints, loiter_radius=2.0):
        """
        waypoints = [(lat,lon), ...]  deg
        loiter_radius = metres (consider reached when < this)
        """
        self.gps = gps
        self.wps = waypoints
        self.idx = 0
        self.loiter = loiter_radius

    def update(self):
        if self.gps.fix == 0 or self.idx >= len(self.wps):
            return None, None   # no fix or mission finished
        tgt_lat, tgt_lon = self.wps[self.idx]
        d, brg = self.gps.dist_bearing(tgt_lat, tgt_lon)
        if d is None:
            return None, None
        if d < self.loiter:
            self.idx += 1
            if self.idx >= len(self.wps):
                return 0, 0      # mission complete → stop
            tgt_lat, tgt_lon = self.wps[self.idx]
            d, brg = self.gps.dist_bearing(tgt_lat, tgt_lon)

        # cross-track steering: +1 = steer right, -1 = left
        error_deg = brg - self.gps.course
        if error_deg > 180:  error_deg -= 360
        if error_deg < -180: error_deg += 360
        steer = np.clip(error_deg / 45.0, -1.0, 1.0)   # 45° = full lock
        # throttle falls with distance (min 0.15 m/s)
        throttle = np.clip(d / 10.0, 0.15, 0.40)
        return throttle, steer