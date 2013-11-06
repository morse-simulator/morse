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
import math
import random
from pymorse import Morse

def send_pose(s, x, y, z, yaw, pitch, roll):
    s.publish({'x' : x, 'y' : y, 'z' : z, 'yaw' : yaw, 'pitch' : pitch, 'roll' : roll})


class TeleportTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = RMax('robot')
        robot.translate(10.0, 8.0, 20.0)
        
        pose = Pose()
        pose.add_stream('socket')
        robot.append(pose)

        teleport = Teleport('teleport')
        teleport.add_stream('socket')
        robot.append(teleport)


        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def _test_one_pose(self, morse, x, y, z, yaw, pitch, roll):
        send_pose(self.teleport_stream, x, y, z, yaw, pitch, roll)
        morse.sleep(0.2)

        pose = self.pose_stream.get()
        self.assertAlmostEqual(pose['yaw'], yaw, delta=self.precision)
        self.assertAlmostEqual(pose['pitch'], pitch, delta=self.precision)
        self.assertAlmostEqual(pose['roll'], roll, delta=self.precision)
        self.assertAlmostEqual(pose['x'], x, delta=self.precision)
        self.assertAlmostEqual(pose['y'], y, delta=self.precision)
        self.assertAlmostEqual(pose['z'], z, delta=self.precision)

    def test_teleport(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            self.pose_stream = morse.robot.pose
            self.teleport_stream = morse.robot.teleport

            self.precision = 0.15

            pose = self.pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=self.precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=self.precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=self.precision)
            self.assertAlmostEqual(pose['x'], 10.0, delta=self.precision)
            self.assertAlmostEqual(pose['y'], 8.0, delta=self.precision)
            self.assertAlmostEqual(pose['z'], 20.0, delta=self.precision)

            # Test only one rotation each time, otherwise, it a bit more
            # complex to check that it does the good transformation
            # (without a matrix transformation library)
            for i in range(0, 5):
                self._test_one_pose(morse,
                                    random.uniform(-30.0, 30.0),
                                    random.uniform(-30.0, 30.0),
                                    random.uniform(10.0, 50.0),
                                    random.uniform(-math.pi, math.pi),
                                    0, 0)
                self._test_one_pose(morse,
                                    random.uniform(-30.0, 30.0),
                                    random.uniform(-30.0, 30.0),
                                    random.uniform(10.0, 50.0),
                                    0,
                                    random.uniform(-math.pi, math.pi),
                                    0)
                self._test_one_pose(morse,
                                    random.uniform(-30.0, 30.0),
                                    random.uniform(-30.0, 30.0),
                                    random.uniform(10.0, 50.0),
                                    0, 0,
                                    random.uniform(-math.pi, math.pi))



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(TeleportTest)
