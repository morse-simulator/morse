#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
from morse.testing.testing import MorseTestCase

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

from pymorse import Morse

class BuilderWheeledRobotTest(MorseTestCase):

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = SegwayRMP400()
        p1 = Pose()
        p1.translate(x=1.0)
        robot.append(p1)

        p2 = Pose()
        robot.append(p2)
        p2.translate(x=2.0)

        robot.add_default_interface('socket')

        env = Environment('empty', fastmode = True)

    def test_correct_position(self):
        """ Tests the simulator can return the list of robots
        
        This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            p1 = morse.robot.p1
            p2 = morse.robot.p2

            delta = 0.05 

            pose1 = p1.get()
            self.assertAlmostEquals(pose1['x'], 1.0, delta=delta)
            self.assertAlmostEquals(pose1['y'], 0.0, delta=delta)
            self.assertAlmostEquals(pose1['yaw'], 0.0, delta=delta)

            pose2 = p2.get()
            self.assertAlmostEquals(pose2['x'], 2.0, delta=delta)
            self.assertAlmostEquals(pose2['y'], 0.0, delta=delta)
            self.assertAlmostEquals(pose2['yaw'], 0.0, delta=delta)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(BuilderWheeledRobotTest)
