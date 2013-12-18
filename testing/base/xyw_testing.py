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

def send_speed(s, morse, x, y, w, t):
    s.publish({'x' : x, 'y' : y, 'w' : w})
    morse.sleep(t)
    s.publish({'x' : 0.0, 'y' : 0.0, 'w' : 0.0})
    morse.sleep(0.1)

class XYW_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = ATRV()

        pose = Pose()
        pose.translate(z=-0.10) # atrv base is 10cm over ground
        robot.append(pose)
        pose.add_stream('socket')

        motion = MotionXYW('motion')
        robot.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_xyw_controller(self):
        with Morse() as morse:

            precision=0.11
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.robot.pose
            pose = pose_stream.get()
            for key, coord in pose.items():
                if key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=precision)

            # xyw socket
            xyw = morse.robot.motion

            send_speed(xyw, morse, 1.0, 0.0, 0.0, 2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 2.0, delta=precision)
            self.assertAlmostEqual(pose['y'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['z'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)

            send_speed(xyw, morse, 0.0, -1.0, 0.0, 2.0)
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 2.0, delta=precision)
            self.assertAlmostEqual(pose['y'], -2.0, delta=precision)
            self.assertAlmostEqual(pose['z'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)

            send_speed(xyw, morse, -1.0, 1.0, 0.0, 2.0)

            pose = pose_stream.get()
            for key, coord in pose.items():
                if key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=precision)

            send_speed(xyw, morse, 1.0, 0.0, -math.pi/4.0, 2.0)
            pose = pose_stream.get()

            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=precision)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=precision)
            self.assertAlmostEqual(pose['z'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)

            send_speed(xyw, morse, 0.5, 0.0, -math.pi/8.0, 12.0)

            pose = pose_stream.get()
            for key, coord in pose.items():
                if key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=2*precision)

            pose = pose_stream.get()
            for key, coord in pose.items():
                if key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=2*precision)

            send_speed(xyw, morse, -2.0, 0.0, math.pi/2.0, 3.0)
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=2*precision)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=2*precision)
            self.assertAlmostEqual(pose['z'], 0.0, delta=2*precision)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=2*precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=2*precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=2*precision)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(XYW_Test)
