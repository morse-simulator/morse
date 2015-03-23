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
from pymorse import Morse

def send_force(s, steer, force, brake):
    s.publish({'steer' : steer, 'force' : force, 'brake' : brake})


class SteerForceTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = Hummer('robot')
        
        pose = Pose()
        pose.add_stream('socket')
        robot.append(pose)

        steer_force = SteerForce()
        robot.append(steer_force)
        steer_force.add_stream('socket')

        env = Environment('land-1/rosace_1', fastmode = True)
        env.set_physics_step_sub(1)
        env.set_camera_location([50.0, -20.0, 50.0])

    def test(self):
        with Morse() as morse:
            pose_stream = morse.robot.pose

            pose = pose_stream.get()
            # It is not really precise to control the robot in this way
            # in open loop. So it is a highly qualitative test.

            x = pose['x']
            y = pose['y']

            # destination socket
            steer_client = morse.robot.steer_force

            send_force(steer_client, 0.0, -20.0, 0.0)
            morse.sleep(3.0)
            send_force(steer_client, 0.0, 0.0, 10.0)
            morse.sleep(1.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], x + 8.5, delta = 1.0)
            self.assertAlmostEqual(pose['y'], y, delta = 1.0)

            # Doubling the force 
            send_force(steer_client, 0.0, -40.0, 0.0)
            morse.sleep(3.0)
            send_force(steer_client, 0.0, 0.0, 10.0)
            morse.sleep(2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], x + 28.0, delta = 1.0)
            self.assertAlmostEqual(pose['y'], y, delta = 1.5)

            # Backward move
            send_force(steer_client, 0.0, 10.0, 0.0)
            morse.sleep(11.0)
            send_force(steer_client, 0.0, 0.0, 10.0)
            morse.sleep(2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], x, delta = 1.5)
            self.assertAlmostEqual(pose['y'], y, delta = 1.0)

            # Turning
            send_force(steer_client, -1.0, -10.0, 0.0)
            morse.sleep(10)
            send_force(steer_client, 0.0, 0.0, 10.0)
            morse.sleep(2.0)
            pose = pose_stream.get()

            self.assertAlmostEqual(pose['yaw'], 2.25, delta = 0.2)
            self.assertAlmostEqual(pose['x'], x - 2.5, delta = 1.0)
            self.assertAlmostEqual(pose['y'], x - 7.0, delta = 1.0)



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(SteerForceTest)
