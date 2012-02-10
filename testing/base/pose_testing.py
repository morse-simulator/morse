#! /usr/bin/env python
"""
This script tests the 'data stream' oriented feature of the socket interface.
"""

from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder.morsebuilder import *
except ImportError:
    pass

import os
import sys
import socket
import json
from pymorse import Morse


class PoseTest(MorseTestCase):

    def setUpEnv(self):
        
        print("Adding a robot...")
        robot = Robot('atrv')
        
        pose = Sensor('pose')
        pose.configure_mw('socket')
        robot.append(pose)

        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')

    def test_base_service_connection(self):
        """ Simply tests if the simulator is reachable by its socket interface.
        """

        morse = Morse()
        morse.close()

    def test_get_pose_streams_service(self):
        """ Tests if we can retrieve the list of published data streams.
        """
        morse = Morse()
        self.assertEquals(set(morse.streams()), set(["Pose"]))
        morse.close()

    def test_read_pose(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        morse = Morse()

        pose_stream = morse.stream('Pose')

        pose = pose_stream.get()

        for coord in pose.values():
            self.assertAlmostEqual(coord, 0.0, 2)

        morse.close()
########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(PoseTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

