import sys
sys.path.append('..')
from utils.camera import Camera
from utils.fall_detection import FallDetection  # David's fall detection is implemented in this class
from utils.running_average import RunningAverage
from utils.image_processing import ImageProcessing as IP
from utils.finite_state_machine import FiniteStateMachine
from utils.current_motion_manager import CurrentMotionManager
from utils.motion_library import MotionLibrary
from controller import Robot, Motion
import cv2


class Eve (Robot):
    NUMBER_OF_DODGE_STEPS = 10

    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.time_step = int(self.getBasicTimeStep())

        self.library = MotionLibrary()
        self.library.add('Cust', './Cust.motion', loop=True)
        self.library.add('Shove', './Shove.motion', loop=True)

        self.leds = {
            'right': self.getDevice('Face/Led/Right'),
            'left':  self.getDevice('Face/Led/Left')
        }

        self.fsm = FiniteStateMachine(
            states=['CHOOSE_ACTION', 'BLOCKING_MOTION'],
            initial_state='CHOOSE_ACTION',
            actions={
                'CHOOSE_ACTION': self.choose_action,
                'BLOCKING_MOTION': self.pending
            }
        )

        self.camera = Camera(self)

        self.fall_detector = FallDetection(self.time_step, self)
        self.current_motion = CurrentMotionManager()
        # load motion files
        self.motions = {
            'SideStepLeft': Motion('../motions/SideStepLeftLoop.motion'),
            'SideStepRight': Motion('../motions/SideStepRightLoop.motion'),
            'TurnRight': Motion('../motions/TurnRight20.motion'),
            'TurnLeft': Motion('../motions/TurnLeft20.motion'),
        }
        self.opponent_position = RunningAverage(dimensions=1)
        self.dodging_direction = 'left'
        self.counter = 0

    def run(self):
        self.leds['right'].set(0xff0000)  # set the eyes to red
        self.leds['left'].set(0xff0000)

        while self.step(self.time_step) != -1:
            self.opponent_position.update_average(
                self._get_normalized_opponent_horizontal_position())
            self.fall_detector.check()
            self.fsm.execute_action()

    def choose_action(self):
        if self.opponent_position.average < -0.4:
            self.current_motion.set(self.motions['TurnLeft'])
            self.library.play('Forwards')
            self.library.play('Cust')
        elif self.opponent_position.average > 0.4:
            self.current_motion.set(self.motions['TurnRight'])
            self.library.play('Forwards')
            self.library.play('Cust')
        else:
            # dodging by alternating between left and right side steps to avoid easily falling off the ring
            if self.dodging_direction == 'left':
                if self.counter < self.NUMBER_OF_DODGE_STEPS:
                    self.current_motion.set(self.motions['SideStepLeft'])
                    self.counter += 1
                else:
                    self.dodging_direction = 'right'
            elif self.dodging_direction == 'right':
                if self.counter > 0:
                    self.current_motion.set(self.motions['SideStepRight'])
                    self.counter -= 1
                else:
                    self.dodging_direction = 'left'
            else:
                return
        self.fsm.transition_to('BLOCKING_MOTION')

    def pending(self):
        # waits for the current motion to finish before doing anything else
        if self.current_motion.is_over():
            self.fsm.transition_to('CHOOSE_ACTION')

    def _get_normalized_opponent_horizontal_position(self):
        """Returns the horizontal position of the opponent in the image, normalized to [-1, 1]
            and sends an annotated image to the robot window."""
        img = self.camera.get_image()
        _, _, horizontal = IP.locate_opponent(img)

        if horizontal is None:
            return 0
        return horizontal * 2 / img.shape[1] - 1

# create the Robot instance and run main loop
wrestler = Eve()
wrestler.run()