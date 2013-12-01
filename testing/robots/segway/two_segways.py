#! /usr/bin/env python
"""
This script tests the Segway RMP400 robot with differential drive actuator
"""

import sys
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def set_speed(s, morse, v, w, t):
    s.set_speed(v, w)
    morse.sleep(t)
    s.stop()
    morse.sleep(1)

class TwoRMP400Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot1 = SegwayRMP400()
        pose1 = Pose()
        robot1.append(pose1)
        pose1.translate(z=-0.1)
        motion1 = MotionVWDiff()
        robot1.append(motion1)
        robot1.add_default_interface('socket')


        robot2 = SegwayRMP400()
        robot2.translate(y=1.0)
        pose2 = Pose()
        robot2.append(pose2)
        pose2.translate(z=-0.1)
        motion2 = MotionVWDiff()
        robot2.append(motion2)
        robot2.add_default_interface('socket')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_motion(self):
        """ Tests that 2 Segway RMP400 in the same scene move
        as expected (as if they were alone).
        This tests the particular wheel parenting mechanism used on
        this robot when several instance are present.
        """
        with Morse() as morse:

            pose1_stream = morse.robot1.pose1
            pose2_stream = morse.robot2.pose2

            pose1_x = pose1_stream.get()['x']
            self.assertAlmostEqual(pose1_x, 0.0, delta=0.03)

            pose2_x = pose2_stream.get()['x']
            self.assertAlmostEqual(pose2_x, 0.0, delta=0.03)

            set_speed(morse.robot1.motion1, morse, 1.0, 0.0, 2.0)
            set_speed(morse.robot2.motion2, morse, 1.0, 0.0, 2.0)

            pose1_x = pose1_stream.get()['x']
            self.assertAlmostEqual(pose1_x, 2.0, delta=0.10)

            pose2_x = pose2_stream.get()['x']
            self.assertAlmostEqual(pose2_x, 2.0, delta=0.10)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(TwoRMP400Test)
