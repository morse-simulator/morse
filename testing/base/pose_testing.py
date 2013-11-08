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

import sys
from pymorse import Morse


class PoseTest(MorseTestCase):

    def setUpEnv(self):
        
        print("Adding a robot...")
        robot = ATRV()
        
        pose = Pose()
        pose.add_stream('socket')
        pose.translate(z=-0.10) # atrv sensor is at 10cm on the groud
        robot.append(pose)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_base_service_connection(self):
        """ Simply tests if the simulator is reachable by its socket interface.
        """

        morse = Morse()
        morse.close()

    def test_get_pose_streams_service(self):
        """ Tests if we can retrieve the list of published data streams.
        """
        morse = Morse()
        self.assertEquals(set(morse.streams()), set(["robot.pose"]))
        morse.close()

    def test_read_pose(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            pose_stream = morse.robot.pose

            pose = pose_stream.get()

            for key, coord in pose.items():
                if key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, 2)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(PoseTest)
