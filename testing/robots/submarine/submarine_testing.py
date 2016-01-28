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
from pymorse import Morse

def send_goal(s, x, y, z):
    s.publish({'x' : x, 'y' : y, 'z' : z})

class DestinationTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = Submarine()
        robot.translate(0.0, 200.0, -20.0)
        
        pose = Pose()
        pose.add_stream('socket')
        robot.append(pose)

        destination = Destination('destination')
        robot.append(destination)
        destination.add_stream('socket')
        destination.properties(Speed=2.0, Tolerance=0.3, RemainAtDestination = True)

        env = Environment('water-1/deep_water', fastmode = True)
        
        water = bpymorse.get_object('Water')
        water.scale = [600.0, 600.0, 0.0]
        env.set_camera_clip(clip_end=1000)
        env.set_camera_location([60,150,150])
        env.set_camera_rotation([.2,0,0])

    def test(self):
        with Morse() as morse:
            pose_stream = morse.robot.pose

            pose = pose_stream.get()
            precision = 0.02
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['y'], 200.0, delta=precision)
            self.assertAlmostEqual(pose['z'], -20.0, delta=0.1)

            dest_client = morse.robot.destination
            send_goal(dest_client, 10.0, 200.0, -20.0)

            morse.sleep(3.0)
            # Only x has changed. Check that speed is respected

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], 3.0 * 2.0, delta=0.1)
            self.assertAlmostEqual(pose['y'], 200.0, delta=precision)
            self.assertAlmostEqual(pose['z'], -20.0, delta=0.1)

            morse.sleep(2.0)
            # Only x has changed
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], 9.7, delta=0.1)
            self.assertAlmostEqual(pose['y'], 200.0, delta=precision)
            self.assertAlmostEqual(pose['z'], -20.0, delta=0.1)

            x = pose['x']
            send_goal(dest_client, x, 210.0, -20.0)
            morse.sleep(5.0)

            # Only Y has changed
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], x, delta=0.1)
            self.assertAlmostEqual(pose['y'], 209.7, delta=0.1)
            self.assertAlmostEqual(pose['z'], -20.0, delta=0.1)


            x = pose['x']
            y = pose['y']
            z = pose['z']

            send_goal(dest_client, x, y, -30.0)
            morse.sleep(5.0)

            # Only Z has changed
            # XXX precision is not really good
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], x, delta=precision)
            self.assertAlmostEqual(pose['y'], y, delta=precision)
            self.assertAlmostEqual(pose['z'], -30.0, delta=0.3)

            send_goal(dest_client, 0, 200, -20)
            morse.sleep(10.0)
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertLess(math.fabs(pose['x']), 0.3)
            self.assertLess(math.fabs(pose['y'] -200), 0.3)
            self.assertLess(math.fabs(pose['z'] + 20), 0.3)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(DestinationTest)
