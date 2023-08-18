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
        # to load all the motions from the motions folder, we use the MotionLibrary class:
        motion_library = MotionLibrary()
        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        while self.step(self.time_step) != -1:  # mandatory function to make the simulation run
            motion_library.play('Forwards')


# create the Robot instance and run main loop
wrestler = Wrestler()
wrestler.run()
