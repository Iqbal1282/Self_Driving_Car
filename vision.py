#! /usr/bin/env python3 
# vision.py 
import cv2 
import numpy as np 
from picamera2 import Picamera2 



class Vision: 
    def __init__(self): 
        self.picam = Picamera2() 

        config = self.picam.create_video_configuration(
            main = {"size": (320, 240), 
                    "format": "RGB888"}
        )

        self.picam.configure(config) 
        self.picam.start()


        # Region of interest (bottom half of image)
        self.roi_x, self.roi_y, self.roi_w, self.roi_h = 0, 120, 320, 120

        # Obstacle detection region (front center)
        self.obstacle_roi = (140, 100, 40, 40)  # x, y, w, h

    def get_frame(self): 
        return self.picam.capture_array()
    
    def detect_lanes(self, frame):
        """Returns steering angle (-1 to 1) based on lane lines"""
        # Crop to road region
        roi = frame[self.roi_y :, :]

        # Convert to grayscale and apply Gaussian blur
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny edge detection
        edges = cv2.Canny(blur, 50, 150)

        # Hough line transform
        lines = cv2.HoughLinesP(
            edges, rho=1, theta=np.pi / 180, threshold=30, minLineLength=20, maxLineGap=10
        )

        if lines is None:
            return 0.0

        # Separate left and right lines
        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 0.001)

            if abs(slope) < 0.3:  # Ignore horizontal lines
                continue

            if slope < 0:  # Right lane (positive slope in image coordinates)
                right_lines.append(line[0])
            else:  # Left lane
                left_lines.append(line[0])

        # Calculate average position for each lane
        


        def avg_position(lines):
            if not lines:
                return None
            x_coords = [x1 for x1, _, x2, _ in lines] + [x2 for _, _, x2, _ in lines]
            return np.mean(x_coords)

        left_x = avg_position(left_lines)
        right_x = avg_position(right_lines)

        # Determine steering angle
        if left_x is not None and right_x is not None:
            # Both lanes detected - aim for center
            center = (left_x + right_x) / 2
            
        elif left_x is not None:
            # Only left lane - stay to the right of it
            center = left_x + 100  # Offset from left lane
        elif right_x is not None:
            # Only right lane - stay to the left of it
            center = right_x - 100  # Offset from right lane
        else:
            return 0.0

        
        image_center = 160
        error = (center - image_center) / 160.0  # Normalize to -1 to 1
        return np.clip(error, -1, 1)
    

    def detect_traffic_light(self, frame): 
        """ Returns 'red', 'yellow', 'green', or none"""

        # Define ROI for traffic light (top portion of image)
        h, w = frame.shape[:2]
        light_roi = frame[: h // 2, :]

        # Convert to HSV color space
        hsv = cv2.cvtColor(light_roi, cv2.COLOR_RGB2HSV)


        ranges = {
            "red": [(0, 100, 100), (10, 255, 255)],
            "yellow": [(20, 100, 100), (30, 255, 255)],
            "green": [(50, 100, 100), (70, 255, 255)],
        }


        detected = "none" 
        max_area = 0 


        for color, (lower, upper) in ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            # Clean up mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100:  # Filter small noise
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect_ratio = h / w if w > 0 else 0

                    # Traffic lights are typically vertical rectangles/circles
                    if aspect_ratio > 1.5 or area > max_area:
                        max_area = area
                        detected = color

        return detected
    

    def detect_obstacle(self, frame):
        """Returns True if obstacle detected in front"""
        x, y, w, h = self.obstacle_roi
        roi = frame[y : y + h, x : x + w]

        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)

        # Apply threshold to find objects
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 200:  # Adjust based on obstacle size
                return True

        return False
    

    def visualize(self, frame, lane_angle, light_state, obstacle):
        """Draw overlays for debugging"""
        # Draw lane ROI
        cv2.rectangle(frame, (self.roi_x, self.roi_y), (self.roi_x + self.roi_w, 240), (0, 255, 0), 1)

        # Draw obstacle ROI
        x, y, w, h = self.obstacle_roi
        color = (0, 0, 255) if obstacle else (255, 0, 0)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

        # Draw lane center line
        center_x = int(160 + lane_angle * 160)
        cv2.line(frame, (center_x, 120), (center_x, 240), (255, 0, 0), 2)

        # Draw traffic light status
        cv2.putText(frame, f"Light: {light_state}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return frame
    
    
    

