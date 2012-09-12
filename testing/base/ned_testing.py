#! /usr/bin/env python
"""
This script tests the 'data stream' oriented feature of the socket interface.
"""

from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import os
import sys
import socket
import json
from pymorse import Morse


class NedTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = Robot('rmax')
        robot.translate(42.0, -10, 40)

        pose = Sensor('pose')
        robot.append(pose)
        pose.configure_mw('socket')

        pose2 = Sensor('pose')
        pose2.configure_modifier('NED')
        robot.append(pose2)
        pose2.configure_mw('socket')


        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')


    def test_ned_pose(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            pose_stream = morse.stream('Pose')
            pose_ned_stream = morse.stream('Pose.001')

            pose = pose_stream.get()
            pose_ned = pose_ned_stream.get()

            self.assertAlmostEqual(pose['x'], 42.0, delta=0.01)
            self.assertAlmostEqual(pose['y'], -10.0, delta=0.01)
            self.assertAlmostEqual(pose['z'], 40.0, delta=0.1)

            self.assertAlmostEqual(pose['x'], pose_ned['y'])
            self.assertAlmostEqual(pose['y'], pose_ned['x'])
            self.assertAlmostEqual(pose['z'], -pose_ned['z'])

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(NedTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

