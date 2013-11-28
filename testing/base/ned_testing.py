#! /usr/bin/env python
"""
This script tests the NED modifier.
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
from pymorse import Morse


class NedTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = RMax('robot')
        robot.translate(42.0, -10, 40)

        teleport = Teleport()
        robot.append(teleport)

        pose = Pose()
        robot.append(pose)
        pose.add_stream('socket')

        pose2 = Pose()
        pose2.alter('NED')
        robot.append(pose2)
        pose2.add_stream('socket')


        env = Environment('empty', fastmode = True)
        env.add_service('socket')


    def test_ned_pose(self):
        """ Test if the NED conversion is correctly done
        """

        with Morse() as morse:
            pose_stream = morse.robot.pose
            pose_ned_stream = morse.robot.pose2

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
    from morse.testing.testing import main
    main(NedTest)
