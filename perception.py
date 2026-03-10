'''
General perception of the robot 
    - Red path detection 
    - Green box detection 
    - Target detection
    - Lego man detection

'''
from itertools import count

import cv2 as cv
import numpy as np
from dataclasses import dataclass

@dataclass 
class SafeZoneEstimate:
    detected: bool
    centroid_x: float
    error_x: float
    error_y: float
    area: float
    # confidence: float

@dataclass
class TargetEstimate:
    detected: bool
    centroid_x: float
    centroid_y: float
    area: float
    # distance_error: float
    # confidence: float
    
@dataclass
class LineEstimate:
    detected: bool
    x_error_center: float
    x_error_ahead: float
    heading_error_ahead: float
    # confidence: float

class OpenCVCamera:
    def __init__(self, camera_index=0, width=640, height=480):
        import cv2
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to capture image from camera")
        return frame

    def release(self):
        self.cap.release()

class Perception:
  
    def __init__(self):
        pass
        
    def detect_red_line(self, frame, lookahead_y=200, focal_length=560.0 ):
        # function to follow the red line (return the cross track error and heading angle error)
        # -------- RED HSV RANGE --------
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        m1 = cv.inRange(hsv, red_lower1, red_upper1)
        m2 = cv.inRange(hsv, red_lower2, red_upper2)
        
        mask = cv.bitwise_or(m1, m2)

        #perform some morphological operations to clean up the mask
        kernel_rect = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel_rect, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel_rect, iterations=2)

        # find the centroid of the line or create a fit line to the points in the center 
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2
        lookahead_y = lookahead_y

        #wants: lateral e at center, lateral e at lookahead, heading error at lookahead
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if not contours:
            return LineEstimate(detected=False, x_error_center=0.0, x_error_ahead=0.0, heading_error_ahead=0.0)

        c = max(contours, key=cv.contourArea)
        if cv.contourArea(c) < 200:
            return LineEstimate(detected=False, x_error_center=0.0, x_error_ahead=0.0, heading_error_ahead=0.0)

        #fitting a line to the contour points reshape for [[x,y], [x,y], ...] to fit a line to 
        pts = c.reshape(-1, 2).astype(np.float32)
        line = cv.fitLine(pts, cv.DIST_L2, 0, 0.01, 0.01)

        # parametric line: (vx, vy) is the direction vector, (x0_fit, y0_fit) is a point on the line
        #x(t) = x0_fit + t*vx
        #y(t) = y0_fit + t*vy -> (y(t) - y0_fit)/vy
        #t is a param to move along the line
        vx, vy, x0_fit, y0_fit = line.flatten()
        vx, vy, x0_fit, y0_fit = float(vx), float(vy), float(x0_fit), float(y0_fit)

        #error at lookahead_y (distance x_center - x @ line at lookahead_y)
        #solve for t when y(t) = lookahead_y
        t_lookahead = (lookahead_y - y0_fit) / vy
        x_error_ahead = center_x - (x0_fit + ((lookahead_y - y0_fit) / vy) * vx)

       
        print("Red line detected at X:", x_at_bottom, " Error at center:", e_norm, " Error at lookahead:", e_px, " Heading error:", theta_e)
        return LineEstimate(detected=True, x_error_center=e_norm, x_error_ahead=e_px, heading_error_ahead=theta_e)
        
        
        # return LineEstimate(detected=False, x_error=0.0, heading_error=0.0)

    def detect_green_box(self, frame, min_area=100):
        # function to detect the green box (return the position and orientation (angle error and distance error)
        # -------- GREEN HSV RANGE --------
        green_lower = np.array([40, 80, 80])
        green_upper = np.array([85, 255, 255])

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, green_lower, green_upper)

        #now theres abinary img of the frame
        #clean it up 
        kernel_ellipse = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel_ellipse, iterations=2)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel_ellipse, iterations=2)

        #get the contours fromthe mask
        contours, _ = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        detected = False
        
        if contours:
            # choose largest contour
            c = max(contours, key=cv.contourArea)
            area = cv.contourArea(c)
            if area > min_area:
                x, y, w, h = cv.boundingRect(c)
                cx = x + w // 2
                cy = y + h // 2
                center = (cx, cy)
             
                # draw detection box and center
                cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

                center_x = frame.shape[1] // 2
                center_y = frame.shape[0] // 2

                error_x = cx - center_x
                error_y = cy - center_y

                print("Green box detected center:", center, "Error:", error_x, "Area:", area)
                return SafeZoneEstimate(detected=True, centroid_x=cx, error_x=error_x, error_y=error_y, area=area)

        return SafeZoneEstimate(detected=False, centroid_x=0.0, error_x=0.0, error_y=0.0, area=0.0)

    def detect_target(self, frame):
        # function to detect the target (return the position and orientation (angle error and distance error)
        
        return False, 0.0, 0.0, frame, None
        
        