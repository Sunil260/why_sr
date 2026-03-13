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
    error_x: float
    error_y: float
    area: float
    # distance_error: float
    # confidence: float
    
@dataclass
class LineEstimate:
    detected: bool
    x_error_center: float #px
    x_error_ahead: float #px
    heading_error_ahead: float #rads
    # confidence: float

class OpenCVCamera:
    def __init__(self, camera_index=0, width=640, height=480, focal_length_px=768.0):
        self.cap = cv.VideoCapture(camera_index)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv.CAP_PROP_FPS, 30)
        self.focal_length_px = focal_length_px  # calced for c27 55def diag fov -> 45.2 Horx FOV -> 640px width -> focal = (w/2) / tan(horzFOV/2)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to capture image from camera")
        return frame

    def release(self):
        self.cap.release()

class Perception:
  
    def __init__(self, camera: OpenCVCamera):
        self.camera = camera
 
    def detect_red_line(self, frame, lookahead_y=200):
        # function to follow the red line (return the cross track error and heading angle error)
        no_res = LineEstimate(detected=False, x_error_center=0.0, x_error_ahead=0.0, heading_error_ahead=0.0)
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
            return no_res

        c = max(contours, key=cv.contourArea)
        if cv.contourArea(c) < 200:
            return no_res

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
        x_error_ahead = center_x - (x0_fit + t_lookahead * vx)

        #use the calculated focal length to estimate the heading error at lookahead_y
        heading_error_ahead = np.arctan2(x_error_ahead, self.camera.focal_length_px)  # angle of the line direction vector

        #main driving error (center to the line at the same row)
        t_center = (center_y - y0_fit) / vy
        x_error_center = center_x - (x0_fit + t_center * vx)
       
        print(f"Red line detected at lookahead Y={lookahead_y}: x_error_ahead={x_error_ahead:.2f}, heading_error_ahead={np.degrees(heading_error_ahead):.2f} degrees")
        return LineEstimate(detected=True, x_error_center=x_error_center, x_error_ahead=x_error_ahead, heading_error_ahead=heading_error_ahead)

    def detect_green_box(self, frame, min_area=100):
        # function to detect the green box (return the position and orientation (angle error and distance error)
        no_res = SafeZoneEstimate(detected=False, centroid_x=0.0, error_x=0.0, error_y=0.0, area=0.0)
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

        return no_res

    def detect_target_cheap(self, frame):
        # function to detect the target asap 

        #shrink image for processing
        #640x480 -> 160x120 for faster processing (keep aspect ratio)
        small_w = 160
        scale = small_w / frame.shape[1]
        small_h = int(frame.shape[0] * scale)
        if small_h <= 0:
            return None

        small = cv.resize(frame, (small_w, small_h), interpolation=cv.INTER_AREA)
        blur = cv.GaussianBlur(small, (5, 5), 0)
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        #blue mask outer ring + some smoothing
        lower_blue = np.array([95, 80, 40])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv.inRange(hsv, lower_blue, upper_blue)

        kernel = np.ones((3, 3), np.uint8)
        blue_mask = cv.morphologyEx(blue_mask, cv.MORPH_OPEN, kernel)
        blue_mask = cv.morphologyEx(blue_mask, cv.MORPH_CLOSE, kernel)

        blue_pixels = cv.countNonZero(blue_mask)
        if blue_pixels < 120:
            return False
        
        #the blue ring is exists
        
        return True

    def analyze_target(self, frame):
        # function to analyze the target (return the position and orientation (angle error and distance error) only if theres a target in frame) 
        no_res = TargetEstimate(detected=False, centroid_x=0.0, centroid_y=0.0, area=0.0, error_x=0.0, error_y=0.0)
        
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # -------- BLUE HSV RANGE --------
        lower_blue = np.array([95, 80, 40])
        upper_blue = np.array([130, 255, 255])

        blue_frame = cv.inRange(hsv, lower_blue, upper_blue)

        # -------- RED HSV RANGE --------
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])
        m_red1 = cv.inRange(hsv, red_lower1, red_upper1)
        m_red2 = cv.inRange(hsv, red_lower2, red_upper2)
        
        red_frame = cv.bitwise_or(m_red1, m_red2)

        #filtering to both 
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

        red_frame = cv.morphologyEx(red_frame, cv.MORPH_OPEN, kernel, iterations=1)
        red_frame = cv.morphologyEx(red_frame, cv.MORPH_CLOSE, kernel, iterations=2)

        blue_frame = cv.morphologyEx(blue_frame, cv.MORPH_OPEN, kernel, iterations=1)
        blue_frame = cv.morphologyEx(blue_frame, cv.MORPH_CLOSE, kernel, iterations=2)


        #find contours - image may have acclusiong try to fit eclipse and return the centroid
        red_countours, _ = cv.findContours(red_frame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        blue_countours, _ = cv.findContours(blue_frame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if not blue_countours:
            return no_res
        
        blue_c = max(blue_countours, key=cv.contourArea)
        red_c = max(red_countours, key=cv.contourArea)

        if cv.contourArea(blue_c) < 100 or cv.contourArea(red_c) < 20:
            return no_res

        #fit the eclipse to both countours and get the centroids
        blue_ellipse = cv.fitEllipse(blue_c)
        red_ellipse = cv.fitEllipse(red_c)

        #centers and axis lengths
        (blue_cx, blue_cy), (blue_w, blue_h), blue_angle = blue_ellipse
        (red_cx, red_cy), (red_w, red_h), red_angle = red_ellipse

        # check centers are close to eachother
        center_dist = np.hypot(blue_cx - red_cx, blue_cy - red_cy)
        if center_dist > 10:
            print("Blue and red contours are not close enough, likely not the target")
            return no_res

        #check area of blue> red
        if blue_w * blue_h < red_w * red_h:
            print("Blue contour area is smaller than red, likely not the target")
            return no_res

        #calculate and report the errors 
        avg_cx = (blue_cx + red_cx) / 2
        avg_cy = (blue_cy + red_cy) / 2
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2

        error_x = avg_cx - center_x
        error_y = avg_cy - center_y


        return TargetEstimate(detected=True, centroid_x=avg_cx, centroid_y=avg_cy, area=blue_w * blue_h, error_x=error_x, error_y=error_y)

    
                               




