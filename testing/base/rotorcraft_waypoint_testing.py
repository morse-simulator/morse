#! /usr/bin/env python
"""
This script tests the waypoints actuator, both the data and service api
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

class RotorcraftWaypoints_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = Quadrotor('robot')
        robot.translate(x = -1.24, y=1.70, z=1.81)

        pose = Pose()
        robot.append(pose)
        pose.add_stream('socket')

        motion = RotorcraftWaypoint('motion')
        robot.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')

        wp_target = Sphere('wp_target')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_waypoint_controller(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
        
            pose_stream = morse.robot.pose
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], -1.24, delta=0.05)
            self.assertAlmostEqual(pose['y'], 1.70, delta=0.05)
            self.assertAlmostEqual(pose['z'], 1.81, delta=0.05)
            self.assertAlmostEqual(pose['yaw'], 0, delta=0.05)


            wp_client = morse.robot.motion
            wp_client.publish({'x' : 10.0, 'y': 5.0, 'z': 10.0,
                                'tolerance' : 0.5, 'yaw' : 1.0})
            morse.sleep(10)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 10.0, delta=0.5)
            self.assertAlmostEqual(pose['y'], 5.0, delta=0.5)
            self.assertAlmostEqual(pose['z'], 10.0, delta=0.5)
            self.assertAlmostEqual(pose['yaw'], 1.0, delta=0.05)


    def test_waypoint_service_controller(self):
        with Morse() as morse:

            pose_stream = morse.robot.pose
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], -1.24, delta=0.05)
            self.assertAlmostEqual(pose['y'], 1.70, delta=0.05)
            self.assertAlmostEqual(pose['z'], 1.81, delta=0.05)
            self.assertAlmostEqual(pose['yaw'], 0, delta=0.05)

            morse.rpc('robot.motion', 'goto', 10.0, 5.0, 10.0, 1.0, 0.5)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 10.0, delta=0.5)
            self.assertAlmostEqual(pose['y'], 5.0, delta=0.5)
            self.assertAlmostEqual(pose['z'], 10.0, delta=0.5)
            self.assertAlmostEqual(pose['yaw'], 1.0, delta=0.05)



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(RotorcraftWaypoints_Test)
