#! /usr/bin/env python
"""
This script tests the human model with a pose sensor.
"""

import sys
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] <your test>.py
try:
    from morse.builder import *
except ImportError:
    pass

class HumanServiceTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        human = Human()
        human.add_service('socket')

        pose = Pose()
        human.append(pose)
        pose.add_stream('socket')

        env = Environment('empty', fastmode = True)


    def test_movement(self):
        """ Tests the human can accept an actuator, and that it
        work as expected to move around the human.

        Currently disabled (the waypoint actuator can not move yet the human)
        """
        with Morse() as morse:

            precision = 0.05

            #Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.human.pose

            pose = pose_stream.get()

            self.assertAlmostEquals(pose['x'], 0.0, delta=precision)
            self.assertAlmostEquals(pose['y'], 0.0, delta=precision)

            morse.human.move(1.0, 0.0)
            morse.sleep(0.1)

            pose = pose_stream.get()

            self.assertAlmostEquals(pose['x'], 1.0, delta=precision)
            self.assertAlmostEquals(pose['y'], 0.0, delta=precision)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(HumanServiceTest)
