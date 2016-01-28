#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
from morse.testing.testing import MorseMoveTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_speed(s, morse, x, y, w, t):
    s.publish({'x' : x, 'y' : y, 'w' : w})
    morse.sleep(t + 0.1)
    s.publish({'x' : 0.0, 'y' : 0.0, 'w' : 0.0})

class XYW_Test(MorseMoveTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = ATRV()

        pose = Pose()
        robot.append(pose)
        pose.add_stream('socket')

        motion = MotionXYW('motion')
        robot.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')

        teleport = Teleport()
        robot.append(teleport)
        teleport.add_stream('socket')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_xyw_controller(self):
        with Morse() as morse:

            morse.deactivate('robot.teleport')
            xyw = morse.robot.motion

            precision = 0.12

            # Read the start position, it must be (0.0, 0.0, 0.0)
            self.assertAlmostEqualPositionThenFix(morse, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            send_speed(xyw, morse, 1.0, 0.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(morse, [2.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            send_speed(xyw, morse, 0.0, -1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(morse, [2.0, -2.0, 0.10, 0.0, 0.0, 0.0], precision)

            send_speed(xyw, morse, -1.0, 1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(morse, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            send_speed(xyw, morse, 1.0, 0.0, -math.pi/4.0, 2.0)
            self.assertAlmostEqualPositionThenFix(morse, [4.0 / math.pi, -4.0 / math.pi, 0.10,
                                 -math.pi / 2.0, 0.0, 0.0], precision)


            send_speed(xyw, morse, 0.5, 0.0, -math.pi/8.0, 12.0)
            self.assertAlmostEqualPositionThenFix(morse, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            send_speed(xyw, morse, -2.0, 0.0, math.pi/2.0, 3.0)
            self.assertAlmostEqualPositionThenFix(morse, [4.0 / math.pi, -4.0 / math.pi, 0.10,
                                 -math.pi / 2.0, 0.0, 0.0], precision*2)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(XYW_Test)
