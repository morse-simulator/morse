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

def send_speed(s, morse, v, w, t):
    s.publish({'v' : v, 'w' : w})
    morse.sleep(t + 0.1)
    s.publish({'v' : 0.0, 'w' : 0.0})

def send_service_speed(s, morse, v, w, t):
    s.set_speed(v, w)
    morse.sleep(t)
    s.stop()

class VW_Test(MorseMoveTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = ATRV()

        pose = Pose()
        robot.append(pose)
        pose.add_stream('socket')

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')

        teleport = Teleport()
        robot.append(teleport)
        teleport.add_stream('socket')
        
        env = Environment('empty', fastmode = True)
        env.show_framerate(True)
        env.add_service('socket')

    def test_vw_controller(self):
        with Morse() as simu:

            simu.deactivate('robot.teleport')

            precision = 0.1
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            self.assertAlmostEqualPositionThenFix(simu, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            v_w = simu.robot.motion

            send_speed(v_w, simu, 1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [2.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            send_speed(v_w, simu, -1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            send_speed(v_w, simu, 1.0, -math.pi/4.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [4.0 / math.pi, -4.0 / math.pi, 0.10,
                                 -math.pi / 2.0, 0.0, 0.0], precision)

            send_speed(v_w, simu, 0.5, -math.pi/8.0, 12.0)
            self.assertAlmostEqualPositionThenFix(simu, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            send_speed(v_w, simu, -2.0, math.pi/2.0, 3.0)
            self.assertAlmostEqualPositionThenFix(simu, [4.0 / math.pi, -4.0 / math.pi, 0.10,
                                 -math.pi / 2.0, 0.0, 0.0], precision*2)

    def test_vw_service_controller(self):
        with Morse() as simu:
            precision = 0.10
            simu.deactivate('robot.teleport')
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            self.assertAlmostEqualPositionThenFix(simu, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            v_w = simu.robot.motion

            send_service_speed(v_w, simu, 1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [2.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            send_service_speed(v_w, simu, -1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            send_service_speed(v_w, simu, 1.0, -math.pi/4.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [4.0 / math.pi, -4.0 / math.pi, 0.10,
                                 -math.pi / 2.0, 0.0, 0.0], precision)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(VW_Test)
