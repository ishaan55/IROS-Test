from controller import Robot
import sys
sys.path.append('..')
from utils.motion_library import MotionLibrary

# Eve's locate_opponent() is implemented in this module:
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
from utils.camera import Camera
from utils.camera2 import Camera2
from utils.finite_state_machine import FiniteStateMachine
from utils.ellipsoid_gait_generator import EllipsoidGaitGenerator


# import torch
import cv2
# from torchvision import transforms
import numpy as np
import time
import threading
    # Load the YOLOv5 model


    # Create an OpenCV window for displaying the YOLOv5 detection results
# cv2.namedWindow('YOLOv5 Detection', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('YOLOv5 Detection', 800, 600)

# model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/ankit/IROS/controllers/participant/recent.pt')
# device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# model.to(device).eval()





class Sultaan (Robot):
    SMALLEST_TURNING_RADIUS = 0.1 #0.1
    SAFE_ZONE = 0.75
    TIME_BEFORE_DIRECTION_CHANGE = 60   # 80
    k=0
    is_bot_visible = True
    
    
    def __init__(self):
        Robot.__init__(self)
        self.fall = Falsed = 0
        
        self.time_step = int(self.getBasicTimeStep())
        self.library = MotionLibrary()
        self.camera = Camera(self)
        self.camera2 = Camera2(self)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        self.counter = 0
        self.library.add('Cust', './Motions/Shove.motion')
        self.leds = {
            'rightf': self.getDevice('Face/Led/Right'), 
            'leftf': self.getDevice('Face/Led/Left'), 
            'righte': self.getDevice('Ears/Led/Right'), 
            'lefte': self.getDevice('Ears/Led/Left'), 
            'chest': self.getDevice('ChestBoard/Led'), 
        }
        
        self.HeadPitch = self.getDevice("HeadPitch")
       
        #self.library.play('Cust')
        # for locking motor
       
    def run(self):
        k=0
        
        counter = 0
        # yolo_thread = threading.Thread(target=self.run_yolo)
        # yolo_thread.start()
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            #self.HeadPitch.setPosition(0)
            t = self.getTime()
            self.leds['rightf'].set(0xff0000)
            self.leds['leftf'].set(0xff0000)
            self.leds['righte'].set(0xff0000)
            self.leds['lefte'].set(0xff0000)
            self.leds['chest'].set(0xff0000)
            self.gait_manager.update_theta()
            self.getRedLineDistance()
            #x, k, z, yaw = EllipsoidGaitGenerator.compute_leg_position(self, is_left = 'True', desired_radius=1e3, heading_angle=0)
            #print('x=' + str(x))

            self.fall = False
            if(self.fall_detector.detect_fall()): 
                self.fall = True

            if 0.3 < t < 2:
                self.start_sequence()
            elif 3 < t < 5:
                self.library.play('TurnLeft60')
            elif t > 5:
                
                self.fall
                self.fall_detector.check()

                if(not self.fall):
                    #print('t_before_yolo: {:.6f}'.format(round(t, 6)))
                    
                    # self.walk()
                    d, floor = self.getRedLineDistance()
                    l = self._get_normalized_opponent_x(1) 
                    self.library.play('Cust')
                    if d == 1:
                    # print("boundary overflow")
                    #prevD = d
                    # self.heading_angle = 3.14 / 2
                        #     self.gait_manager.command_to_motors(heading_angle=0)
                        # else:
                        if floor not in ['left', 'right', -1]:
                            # self.gait_manager.command_to_motors(desired_radius=0.1 ,heading_angle=(3.14)/2)
                            # continue
                            # print('p')
                            self.library.play('TurnLeft60')
                            # self.gait_manager.command_to_motors(desired_radius=0, heading_angle=-self.heading_angle)
                    else:
                        self.walk()
    
    def getFloorDirection(self,image):
        THRESHOLD = 45
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        m,n = image_gray.shape
        top_row = np.mean(image_gray[0])
        bottom_row = np.mean(image_gray[image_gray.shape[0]-1])
        left_column = np.mean(image_gray[:,0])
        right_column = np.mean(image_gray[:,image_gray.shape[1]-1])
        if top_row - bottom_row > THRESHOLD:
            return 'bottom'
        elif bottom_row - top_row > THRESHOLD:
            return 'top'
        elif left_column - right_column > THRESHOLD:
            return 'right'
        elif right_column - left_column > THRESHOLD:
            return 'left'
        else:
            return -1
        
    def ring_pos(self):
        image = self.camera2.get_image()
        hsv_image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        img1 = hsv_image.copy()
        img2 = hsv_image.copy()
        m = 0
        # colorr_low = 168  
        # colorr_high = 209
        # colorf_low = 70  
        # colorf_high = 102
        # lower_red = (193, 62, 35)
        # upper_red = (205, 107, 65)
        colorr_low = np.array([193,62,35])
        colorr_high = np.array([205,107,65])
        colorf_low = np.array([83,62,42])
        colorf_high = np.array([154,110,70])
        mask1 = cv2.inRange(img1, colorr_low, colorr_high)
        mask2 = cv2.inRange(img2, colorf_low, colorf_high)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel)
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel)
        res1 = cv2.bitwise_and(img1,img1,mask1)
        res2 =  cv2.bitwise_and(img2,img2,mask2)
        gray1 = cv2.cvtColor(res1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(res2, cv2.COLOR_BGR2GRAY)
        contours1, _ = cv2.findContours(gray1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours2, _ = cv2.findContours(gray2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours1 = sorted(contours1, key=cv2.contourArea, reverse=True)
        contours2 = sorted(contours2, key=cv2.contourArea, reverse=True)
        # Check if contours2 is non-zero before calculating its centroid
        cy1, cx1 = None, None
        if len(contours1) > 0:
            contours1 = sorted(contours1, key=cv2.contourArea, reverse=True)
            cy1, cx1 = IP.get_contour_centroid(contours1[0])
        # Check if contours2 is non-zero before calculating its centroid
        cy2, cx2 = None, None
        if len(contours2) > 0:
            contours2 = sorted(contours2, key=cv2.contourArea, reverse=True)
            cy2, cx2 = IP.get_contour_centroid(contours2[0])

        # mask_red = cv2.inRange(hsv_image , lower_red , upper_red)
        # res = cv2.bitwise_and(hsv_image, hsv_image, mask_red)
        # gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        # edged = cv2.Canny(gray, 20, 100)
        # contours_red , _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # contours = sorted(contours_red, key=cv2.contourArea, reverse=True)[:1]
        # # Calculate slope of the bounding rectangle
        # if len(contours_red)>0:
        #     x, y, w, h = cv2.boundingRect(contours[0])
        #     slope = h / w

        # cy3, cx3 = None, None
        # if len(contours_red) > 0:
        #     contours_red = sorted(contours_red, key=cv2.contourArea, reverse=True)
        #     cy3, cx3 = IP.get_contour_centroid(contours_red[0])



        # Continue with your code using cy3 and cx3, knowing that they are either valid centroids or None if contours_red is empty.

        # Assuming you have already calculated cy1, cx1, cy2, and cx2

        if len(contours1) > 0 and len(contours2) > 0:
            if cy1 > cy2:
                return 0
            else:
                return 1
        #     elif (cx1 > 108 and cy1 <40):
        #         return 1
        #     elif (cx1 < 50 and cy1 < 40):
        #         return 2
        #     elif (slope <= 0.3):
        #         return 3
        #     elif (slope > 0.3 and cx1 < 80):
        #         return 4
        #     elif (slope >= 0.3 and cx1 > 80):
        #         return 5
        #     elif (cx1 == 0 and cy1 == 0):
        #         return 6
        # if len(contours1)==0:
        #      return 0
    # problem abhi bhi hai F

    # Handle the case when either or
        
    def red_slope(self):
        image = self.camera2.get_image()
        hsv_image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        # lower_red = np.array([193, 62, 35])
        # upper_red = np.array([205, 107, 65])
        # mask_red = cv2.inRange(hsv_image , lower_red , upper_red)
        # res = cv2.bitwise_and(hsv_image, hsv_image, mask_red)
        # gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        # edged = cv2.Canny(gray, 20, 100)
        # contours_red , _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # contours = sorted(contours_red, key=cv2.contourArea, reverse=True)[:1]
        # # Calculate slope of the bounding rectangle
        # if len(contours_red)>0:
        #     x, y, w, h = cv2.boundingRect(contours[0])
        #     slope = h / w

        # if(slope>0.30):
        #     return 0
        # else:
        #     return 1 
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)

        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours_red, key=cv2.contourArea, reverse=True)[:1]

        # for contour in contours:
        #     area = cv2.contourArea(contour)

        #     if area > 2500:
        #         x, y, w, h = cv2.boundingRect(contour)
        #         slope = h / w

        #         if(slope<0.3):
        #             return 1
        #         else:
        #             return 0     
        # Initialize the rotated bounding rectangle
        rotated_rect = None

# Loop through the detected contours
        for contour in contours:
    # Fit a rotated bounding rectangle around the contour
            rotated_rect = cv2.minAreaRect(contour)
            if(rotated_rect[2]>75 and rotated_rect[2]<105):
                return 1
            else:
                return 0 

    
    def getRedLineDistance(self):          #we use bottom oriented image for edge detection
        import cv2
        import numpy as np
        image = self.camera2.get_image()
        floor = self.getFloorDirection(image)
        m = 0
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = (0, 50, 50)
        upper_red = (10, 255, 255)
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
       
        
        rgb_image = image[:, :, :3]

# Get the shape of the RGB image
        rgb_image_shape = rgb_image.shape

# Get the number of channels from the shape tuple
        num_channels = rgb_image_shape[-1]
        # print('num_channels:', num_channels)
        
        
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)

            image_height, image_width = image.shape[:2]
            bottom_threshold = 0.92 * image_height
            
            for point in box:
                x, y = point
                if y >= bottom_threshold:
                    pass
                    # print("Point:", point)
                    # print("Bottom Threshold:", bottom_threshold)

            points_below_threshold = sum(point[1] >= bottom_threshold for point in box)
            percentage_below_threshold = points_below_threshold / len(box)
            
            #if any(point[1] >= bottom_threshold for point in box):
            cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
            # self.camera2.send_to_robot_window(image)
            # print('percentage_below_threshold: ', percentage_below_threshold)
            if percentage_below_threshold >= 0.5:    #print('point[1]: ', point)
                #print('bottom_threshold: ', bottom_threshold)
                if cv2.contourArea(largest_contour) >= 200:
                    # print("Turn to avoid falling!")
                    m=1
                
                else:
                    pass
                    # print("No need to turn, keep moving.")
            else:
                pass
                # print("No need to turn, keep moving.")
        else:
            pass
            # print("No red contours found, keep moving.")
        return m, floor

    
    
    # def run_yolo(self):
    #     time.sleep(2)
    #     while True:
    #         # Capture the image from the camera
    #         image = self.camera.get_image()

    #         # Remove alpha channel if present
    #         if image.shape[2] == 4:
    #             image = image[:, :, :3]

    #         # Convert image to RGB format
    #         img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    #         # Perform object detection
    #         results = model([img])

    #         # Display the detections
    #         results.print()

    #         # Access individual detection attributes (e.g., bounding boxes, labels)
    #         boxes = results.xyxy[0].numpy()
    #         labels = results.names[0]

    #         # Process the detection results as needed
    #         if len(boxes) == 0:
    #             is_bot_visible = False
    #             self.library.play('TurnLeft60')
    #         else:
    #             is_bot_visible = True
                

    #         # You can perform further actions based on the detection results

    #         # Sleep for a short duration to avoid excessive CPU usage
    #         time.sleep(0.1)
    
    
  
    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        self.gait_manager.command_to_motors(heading_angle=0)
        
    
    
    def walk(self):
        normalized_x,_ = self._get_normalized_opponent_x() 
        desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x) if abs(normalized_x) > 1e-3 else None
        if(normalized_x > 0): 
            self.heading_angle = 3.14/4
            self.counter = 0;  
        elif(normalized_x < 0): 
            self.heading_angle = -(3.14/4)
            self.counter = 0 
        elif(normalized_x == 0): 
            return  
        self.counter += 1
        self.gait_manager.command_to_motors(desired_radius=desired_radius, heading_angle=self.heading_angle)
        # self.library.play('Cust')

    def _get_normalized_opponent_x(self, type=0):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        l, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0,None
        if type:
            return l
        return horizontal_coordinate * 2 / img.shape[1] - 1, l

# create the Robot instance and run main loop
wrestler = Sultaan()
wrestler.run()
