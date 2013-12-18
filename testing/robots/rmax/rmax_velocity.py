#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_speed(s, morse, v, w, t):
    s.publish({'v' : v, 'w' : w})
    morse.sleep(t)
    s.publish({'v' : 0.0, 'w' : 0.0})


class RMaxVelocity_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot = RMax()
        robot.translate(z = 10.0)

        pose = Pose()
        robot.append(pose)
        pose.add_stream('socket')

        motion = MotionVW()
        motion.properties(ControlType = 'Velocity')
        robot.append(motion)
        motion.add_stream('socket')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_vw_controller(self):
        with Morse() as simu:

            Z = 10.0

            precision=0.10
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = simu.robot.pose
            pose = pose_stream.get()
            for key, coord in pose.items():
                if key != 'timestamp' and key != 'z':
                    self.assertAlmostEqual(coord, 0.0, delta=precision)


            v_w = simu.robot.motion

            send_speed(v_w, simu, 1.0, 0.0, 2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 2.0, delta=precision)
            self.assertAlmostEqual(pose['y'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['z'], Z, delta=precision)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)

            send_speed(v_w, simu, -1.0, 0.0, 2.0)

            pose = pose_stream.get()
            for key, coord in pose.items():
                if key != 'timestamp' and key != 'z':
                    self.assertAlmostEqual(coord, 0.0, delta=precision)

            send_speed(v_w, simu, 1.0, -math.pi/4.0, 2.0)
            pose = pose_stream.get()

            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=precision)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=precision)
            self.assertAlmostEqual(pose['z'], Z, delta=precision)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)

            send_speed(v_w, simu, 0.5, -math.pi/8.0, 12.0)

            pose = pose_stream.get()
            for key, coord in pose.items():
                if key != 'timestamp' and key != 'z':
                    self.assertAlmostEqual(coord, 0.0, delta=precision)

            send_speed(v_w, simu, -2.0, math.pi/2.0, 3.0)
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=0.1)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.1)
            self.assertAlmostEqual(pose['z'], Z, delta=0.1)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.1)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.1)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(RMaxVelocity_Test)
