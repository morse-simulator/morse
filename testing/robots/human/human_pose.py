#! /usr/bin/env python
"""
This script tests the human model with a pose sensor.
"""

import sys
from time import sleep
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] <your test>.py
try:
    from morse.builder import *
except ImportError:
    pass

class HumanPoseTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        human = Human()

        pose = Pose()
        human.append(pose)
        pose.add_stream('socket')

        motion = Waypoint()
        human.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_pose(self):
        """ Tests we can load the human model, attach a pose sensor, and
        get back the pose.
        """
        with Morse() as morse:

            #Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.human.pose
            sleep(1)
            pose = pose_stream.get()
            for coord in pose.values():
                self.assertAlmostEqual(coord, 0.0, delta=0.1)

    def _test_movement(self):
        """ Tests the human can accept an actuator, and that it
        work as expected to move around the human.

        Currently disabled (the waypoint actuator can not move yet the human)
        """
        with Morse() as morse:

            #Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.human.pose

            # waypoint controller socket
            v_w_client = morse.human.motion

            v_w_client.publish({'x' : 2.0, 'y': 3.0, 'z': 0.0,
                                'tolerance' : 0.3,
                                'speed' : 1.0})

            sleep(5)
            pose = pose_stream.get()

            self.assertAlmostEqual(pose['x'], 2.0, delta=0.5)
            self.assertAlmostEqual(pose['y'], 3.0, delta=0.5)


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(HumanPoseTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())
