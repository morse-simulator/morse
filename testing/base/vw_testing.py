#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
from time import sleep
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_speed(s, v, w, t):
    s.publish({'v' : v, 'w' : w})
    sleep(t)
    s.publish({'v' : 0.0, 'w' : 0.0})

def send_service_speed(s, v, w, t):
    s.set_speed(v, w)
    sleep(t)
    s.stop()

class VW_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = ATRV()

        pose = Pose()
        pose.translate(z=-0.10) # atrv base is 10cm over ground
        robot.append(pose)
        pose.add_stream('socket')

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_vw_controller(self):
        with Morse() as simu:

            precision=0.05
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = simu.robot.pose
            pose = pose_stream.get()
            for coord in pose.values():
                self.assertAlmostEqual(coord, 0.0, delta=precision)

            v_w = simu.robot.motion

            send_speed(v_w, 1.0, 0.0, 2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 2.0, delta=precision)
            self.assertAlmostEqual(pose['y'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['z'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)

            send_speed(v_w, -1.0, 0.0, 2.0)

            pose = pose_stream.get()
            for coord in pose.values():
                self.assertAlmostEqual(coord, 0.0, delta=precision)

            send_speed(v_w, 1.0, -math.pi/4.0, 2.0)
            pose = pose_stream.get()

            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=precision)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=precision)
            self.assertAlmostEqual(pose['z'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)

            send_speed(v_w, 0.5, -math.pi/8.0, 12.0)

            pose = pose_stream.get()
            for coord in pose.values():
                self.assertAlmostEqual(coord, 0.0, delta=precision)

            send_speed(v_w, -2.0, math.pi/2.0, 3.0)
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=0.1)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.1)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.1)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.1)

    def test_vw_service_controller(self):
        with Morse() as simu:
            precision=0.15
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = simu.robot.pose
            pose = pose_stream.get()
            for coord in pose.values():
                self.assertAlmostEqual(coord, 0.0, delta=precision)

            v_w = simu.robot.motion

            send_service_speed(v_w, 1.0, 0.0, 2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 2.0, delta=precision)
            self.assertAlmostEqual(pose['y'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['z'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)

            send_service_speed(v_w, -1.0, 0.0, 2.0)

            pose = pose_stream.get()
            for coord in pose.values():
                self.assertAlmostEqual(coord, 0.0, delta=precision)

            send_service_speed(v_w, 1.0, -math.pi/4.0, 2.0)
            pose = pose_stream.get()

            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=precision)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=precision)
            self.assertAlmostEqual(pose['z'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(VW_Test)
