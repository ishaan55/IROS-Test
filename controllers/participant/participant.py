"""Minimalist controller example for the Robot Wrestling Tournament.
   Demonstrates how to play a simple motion file."""

from controller import Robot
import sys

sys.path.append('..')
from utils.motion_library import MotionLibrary
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
# from utils.gait_manager import GaitManager
from utils.camera import Camera


class Wrestler (Robot):
    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = Camera(self)
        self.fall_detector = FallDetection(self.time_step, self)
        # self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        # Time before changing direction to stop the robot from falling off the ring
        self.counter = 0

    def run(self):
        motion_library = MotionLibrary()
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            t = self.getTime()
            # self.gait_manager.update_theta()
            if 0.3 < t < 2:
                motion_library.play('Forwards')
            elif t > 2:
                self.fall_detector.check()
                motion_library.play('Forwards')

            


# create the Robot instance and run main loop
wrestler = Wrestler()
wrestler.run()
