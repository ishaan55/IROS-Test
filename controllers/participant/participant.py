from controller import Robot
import sys
sys.path.append('..')
# Eve's locate_opponent() is implemented in this module:
from utils.motion_library import MotionLibrary
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
from utils.camera import Camera


class Wrestler (Robot):
    SMALLEST_TURNING_RADIUS = 0.1
    SAFE_ZONE = 0.75
    TIME_BEFORE_DIRECTION_CHANGE = 60  # 8000 ms / 40 ms

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.library = MotionLibrary()
        self.camera = Camera(self)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        # Time before changing direction to stop the robot from falling off the ring
        self.counter = 0

    def run(self):
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            t = self.getTime()
            self.gait_manager.update_theta()
            if 0.3 < t < 2:
                self.start_sequence()
            elif t > 2:
                self.fall_detector.check()
                self.walk()
                self.library.play('Cust')

    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        self.gait_manager.command_to_motors(heading_angle=0)

    def walk(self):
        """Dodge the opponent robot by taking side steps."""
        normalized_x = self._get_normalized_opponent_x()
        # We set the desired radius such that the robot walks towards the opponent.
        # If the opponent is close to the middle, the robot walks straight.
        desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x) if abs(normalized_x) > 1e-3 else None
        # TODO: position estimation so that if the robot is close to the edge, it switches dodging direction
        if self.counter > self.TIME_BEFORE_DIRECTION_CHANGE:
            self.heading_angle = - self.heading_angle
            self.counter = 0
        self.counter += 1
        self.gait_manager.command_to_motors(desired_radius=desired_radius, heading_angle=self.heading_angle)

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2 / img.shape[1] - 1
    
# create the Robot instance and run main loop
wrestler = Wrestler()
wrestler.run()

# """Minimalist controller example for the Robot Wrestling Tournament.
#    Demonstrates how to play a simple motion file."""

# from controller import Robot
# import sys

# # We provide a set of utilities to help you with the development of your controller. You can find them in the utils folder.
# # If you want to see a list of examples that use them, you can go to https://github.com/cyberbotics/wrestling#demo-robot-controllers
# sys.path.append('..')
# from utils.motion_library import MotionLibrary
# from utils.image_processing import ImageProcessing as IP
# from utils.fall_detection import FallDetection
# # from utils.gait_manager import GaitManager
# from utils.camera import Camera


# class Wrestler (Robot):
#     SMALLEST_TURNING_RADIUS = 0.1 #0.1
#     SAFE_ZONE = 0.75
#     TIME_BEFORE_DIRECTION_CHANGE = 60   # 80
#     k=0
#     is_bot_visible = True

#     def __init__(self):
#         Robot.__init__(self)
#         self.fall = Falsed = 0
        
#         self.time_step = int(self.getBasicTimeStep())
#         self.library = MotionLibrary()

#         self.camera = Camera(self)
#         # self.camera2 = Camera2(self)
#         self.fall_detector = FallDetection(self.time_step, self)
#         # self.gait_manager = GaitManager(self, self.time_step)
#         self.heading_angle = 3.14 / 2
#         self.counter = 0

#     def run(self):

#         while self.step(self.time_step) != -1:
#             # We need to update the internal theta value of the gait manager at every step:
#             t = self.getTime()
#             if(self.fall_detector.detect_fall()): 
#                 self.fall = True
#             # self.gait_manager.update_theta()
#             if 0.3 < t < 2:
#                 self.library.play('Forwards')
#             elif t > 2:
#                 self.fall_detector.check()

#                 self.library.play('Forwards')
#                 self.library.play('Cust')

#         def _get_normalized_opponent_x(self):
# #         """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
#             img = self.camera.get_image()
#             _, _, horizontal_coordinate = IP.locate_opponent(img)
#             if horizontal_coordinate is None:
#                 return 0
#             return horizontal_coordinate * 2 / img.shape[1] - 1


# # create the Robot instance and run main loop
# wrestler = Wrestler()
# wrestler.run()