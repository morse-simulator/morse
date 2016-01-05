#! /usr/bin/env python
"""
This script tests the basics of the human model.
"""

import sys
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|edit] <your test>.py
try:
    from morse.builder import *
except ImportError:
    pass

class HumanBaseTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        human = Human()

        pose = Pose()
        human.append(pose)
        pose.add_stream('socket')

        #motion = Keyboard()

        motion = Waypoint()
        motion.properties(ControlType='Position')
        motion.add_stream('socket')
        motion.add_service('socket')

        human.append(motion)

        env = Environment('empty', fastmode=True)
        env.add_service('socket')

    def test_movement(self):
        """ Tests the human can accept an actuator, and that it
        work as expected to move around the human.
        """
        with Morse() as morse:

            #Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.human.pose
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.1)


            # waypoint controller socket
            human_ctl = morse.human.motion

            human_ctl.publish({'x' : 2.0, 'y': 3.0, 'z': 0.0,
                               'tolerance' : 0.3,
                               'speed' : 1.0})

            morse.sleep(5)
            pose = pose_stream.get()

            self.assertAlmostEqual(pose['x'], 2.0, delta=0.5)
            self.assertAlmostEqual(pose['y'], 3.0, delta=0.5)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(HumanBaseTest)
