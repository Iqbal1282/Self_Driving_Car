#!/usr/bin/env python3
import cv2, numpy as np
from picamera2 import Picamera2   # Pi OS Bullseye/Bookworm

class Vision:
    def __init__(self, resolution=(320, 240), fps=30):
        self.picam = Picamera2()
        # ArduCam OV5647 full-sensor → down-scale in hardware
        config = self.picam.create_video_configuration(
            main={"size": resolution, "format": "RGB888"},
            raw={"size": (1296, 972)},  # 2×2 binning = less noise
            controls={"FrameRate": fps, "AnalogueGain": 1.5}
        )
        self.picam.configure(config)
        # Auto-exposure / auto-white-balance (good for outdoor)
        self.picam.set_controls({"AeEnable": True, "AwbEnable": True})
        self.picam.start()

        # same ROIs as before
        self.roi_y, self.roi_h = 120, 120
        self.obs_roi = (140, 100, 40, 40)

    # ---------- grab frame ----------
    def get_frame(self):
        return self.picam.capture_array()

    # ---------- lane ----------
    def detect_lanes(self, frame):
        roi = frame[self.roi_y:, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, minLineLength=20, maxLineGap=10)
        if lines is None:
            return 0.0
        left, right = [], []
        for l in lines:
            x1, y1, x2, y2 = l[0]
            slope = (y2-y1)/(x2-x1+1e-6)
            if abs(slope) < 0.3:
                continue
            (left if slope < 0 else right).append(l[0])
        # same centre-line maths as before
        def avg_x(lines):
            if not lines:
                return None
            xs = [x1 for x1, _, x2, _ in lines] + [x2 for _, _, x2, _ in lines]
            return np.mean(xs)
        l_x, r_x = avg_x(left), avg_x(right)
        if l_x is None or r_x is None:
            return 0.0
        mid_top = (l_x + r_x) / 2
        mid_bot = (l_x + r_x) / 2   # simplified
        angle = np.arctan2(mid_top - mid_bot, 60)
        return float(np.clip(angle * 2, -0.6, 0.6))

    # ---------- traffic-light colours ----------
    def detect_traffic_light(self, frame):
        hsv = cv2.cvtColor(frame[:frame.shape[0]//2, :], cv2.COLOR_RGB2HSV)
        ranges = {"red": [(0, 100, 100), (10, 255, 255)],
                  "yellow": [(20, 100, 100), (30, 255, 255)],
                  "green": [(50, 100, 100), (70, 255, 255)]}
        best_colour, best_area = "nothing", 0
        for colour, (low, high) in ranges.items():
            mask = cv2.inRange(hsv, np.array(low), np.array(high))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area > best_area:
                    best_area = area
                    best_colour = colour
        return best_colour if best_area > 100 else "nothing"

    # ---------- obstacle (front patch) ----------
    def detect_obstacle(self, frame):
        x, y, w, h = self.obs_roi
        roi = frame[y:y + h, x:x + w]
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        _, mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) > 200:
                return True
        return False